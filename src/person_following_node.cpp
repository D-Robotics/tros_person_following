// Copyright (c) 2024，D-Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "tros_person_following/person_following_node.h"

#include <angles/angles.h>
#include <nav2_util/geometry_utils.hpp>
#include <nav2_util/robot_utils.hpp>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <memory>
#include <queue>
#include <utility>

using namespace std::placeholders;

namespace tros_person_following
{

// Nav2 Costmap2D publishes its raw costs (unsigned char, 0-254) via
// nav_msgs/OccupancyGrid whose data is int8_t. LETHAL_OBSTACLE = 254 and
// NO_INFORMATION = 255; as signed int8_t these read as -2 and -1, so any
// comparison must first cast through uint8_t (see cellCost). Local constexpr —
// the project does not depend on costmap_2d headers.
constexpr int kLethalObstacleCost = 254;

PersonFollowingNode::PersonFollowingNode(const rclcpp::NodeOptions & options)
: Node("tros_person_following_node", options)
, tf_buffer_(this->get_clock())
, tf_listener_(tf_buffer_)
, tp_target_lost_(this->now())
, tp_lost_(this->now())
, tp_target_find_start_(this->now())
{
  // Declare parameters
  // --- Topics & Frames ---
  this->declare_parameter<std::string>("detect_result_topic_name", "/tros_seg_fusion");
  this->declare_parameter<std::string>("goal_pose_topic_name", "nearest_pose");
  this->declare_parameter<std::string>("global_frame", "map");
  this->declare_parameter<std::string>("robot_frame", "base_footprint");
  this->declare_parameter<std::string>("followed_target_topic", "tros_person_followed");
  this->declare_parameter<std::string>("followed_target_pose_topic", "tros_followed_target_pose");
  this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  this->declare_parameter<std::string>("costmap_topic", "/global_costmap/costmap");

  // --- Target filtering ---
  this->declare_parameter<float>("target_filter_width_thr", target_filter_width_thr_);
  this->declare_parameter<float>("target_filter_height_thr", target_filter_height_thr_);
  this->declare_parameter<float>("target_filter_range_x_min", 0.1);
  this->declare_parameter<float>("target_filter_range_x_max", 2.0);
  this->declare_parameter<float>("target_filter_range_y_min", -1.0);
  this->declare_parameter<float>("target_filter_range_y_max", 1.0);
  this->declare_parameter<float>("target_filter_confidence_thr", 0.5);

  // --- Target selection ---
  this->declare_parameter<double>("select_min_confidence", 0.7);
  this->declare_parameter<float>("select_weight_dist", 0.5);
  this->declare_parameter<float>("select_weight_score", 0.3);
  this->declare_parameter<float>("select_weight_center", 0.2);

  // --- Follow distance band ---
  this->declare_parameter<double>("follow_distance_min", 1.0);
  this->declare_parameter<double>("follow_distance_max", 2.0);
  this->declare_parameter<double>("follow_hysteresis", 0.1);
  this->declare_parameter<double>("follow_min_safe_distance", 0.5);
  this->declare_parameter<double>("follow_goal_pub_rate", 2.5);
  this->declare_parameter<float>("follow_goal_dist_deadzone", 0.3);
  this->declare_parameter<float>("follow_goal_yaw_deadzone", 0.6);

  // --- Edge-of-frame turn ---
  this->declare_parameter<double>("image_width", 640.0);
  this->declare_parameter<double>("edge_margin_ratio", 0.15);

  // --- Stationary-target switch ---
  this->declare_parameter<double>("static_target_timeout_sec", 8.0);
  this->declare_parameter<double>("static_target_move_thr", 0.2);
  this->declare_parameter<double>("static_switch_depth_diff_thr", 0.3);
  this->declare_parameter<double>("static_switch_activity_window_sec", 2.0);

  // --- Prediction ---
  this->declare_parameter<double>("predict_window_sec", 4.0);
  this->declare_parameter<double>("predict_lead_sec", 2.0);
  this->declare_parameter<double>("predict_max_dist", 3.0);
  this->declare_parameter<double>("predict_stale_sec", 5.0);

  // --- IDLE search & observe ---
  this->declare_parameter<float>("idle_search_start_timeout_sec", 5.0);
  this->declare_parameter<float>("idle_search_total_timeout_sec", 30.0);
  this->declare_parameter<double>("idle_observe_duration_sec", 3.0);
  this->declare_parameter<double>("observe_cooldown_sec", 3.0);
  this->declare_parameter<float>("spin_radian", spin_radian_);
  this->declare_parameter<double>("explore_spin_angular_speed", explore_spin_angular_speed_);

  // --- LOST recovery / belief search / relock ---
  this->declare_parameter<float>("tracking_to_lost_timeout_sec", 1.0);
  this->declare_parameter<float>("lost_to_idle_timeout_sec", 5.0);
  // --- 蜂鸣器节流：最小发声间隔（秒），压制 TRACKING↔LOST 闪烁 chatter ---
  this->declare_parameter<double>("buzzer_min_interval_sec", 3.0);
  this->declare_parameter<double>("belief_search_timeout_sec", 8.0);
  this->declare_parameter<int>("belief_search_max_rounds", 2);
  // LOST 持续时间分段的 relock 距离阈值（早期严格、晚期宽松）。
  this->declare_parameter<double>("relock_dist_phase1", 1.5);
  this->declare_parameter<double>("relock_dist_phase2", 3.0);
  this->declare_parameter<double>("relock_dist_phase3", 5.0);
  this->declare_parameter<double>("relock_dist_phase1_sec", 2.0);
  this->declare_parameter<double>("relock_dist_phase2_sec", 5.0);

  // --- Misc ---
  // Costmap free-cell check for goal placement.
  this->declare_parameter<double>("costmap_free_cost_thr", 50.0);
  this->declare_parameter<double>("costmap_free_search_radius_m", 2.0);

  // Get parameters
  // --- Topics & Frames ---
  detect_result_topic_name_ = this->get_parameter("detect_result_topic_name").as_string();
  goal_pose_topic_name_ = this->get_parameter("goal_pose_topic_name").as_string();
  global_frame_ = this->get_parameter("global_frame").as_string();
  robot_frame_ = this->get_parameter("robot_frame").as_string();
  followed_target_topic_ = this->get_parameter("followed_target_topic").as_string();
  followed_target_pose_topic_ = this->get_parameter("followed_target_pose_topic").as_string();
  cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
  costmap_topic_ = this->get_parameter("costmap_topic").as_string();
  // --- Target filtering ---
  target_filter_width_thr_ = this->get_parameter("target_filter_width_thr").as_double();
  target_filter_height_thr_ = this->get_parameter("target_filter_height_thr").as_double();
  target_filter_range_x_min_ = this->get_parameter("target_filter_range_x_min").as_double();
  target_filter_range_x_max_ = this->get_parameter("target_filter_range_x_max").as_double();
  target_filter_range_y_min_ = this->get_parameter("target_filter_range_y_min").as_double();
  target_filter_range_y_max_ = this->get_parameter("target_filter_range_y_max").as_double();
  target_filter_confidence_thr_ = this->get_parameter("target_filter_confidence_thr").as_double();
  // --- Target selection ---
  select_min_confidence_ = this->get_parameter("select_min_confidence").as_double();
  select_weight_dist_ = this->get_parameter("select_weight_dist").as_double();
  select_weight_score_ = this->get_parameter("select_weight_score").as_double();
  select_weight_center_ = this->get_parameter("select_weight_center").as_double();
  // --- Follow distance band ---
  follow_distance_min_ = this->get_parameter("follow_distance_min").as_double();
  follow_distance_max_ = this->get_parameter("follow_distance_max").as_double();
  follow_hysteresis_ = this->get_parameter("follow_hysteresis").as_double();
  follow_min_safe_distance_ = this->get_parameter("follow_min_safe_distance").as_double();
  follow_goal_pub_rate_ = this->get_parameter("follow_goal_pub_rate").as_double();
  follow_goal_dist_deadzone_ = this->get_parameter("follow_goal_dist_deadzone").as_double();
  follow_goal_yaw_deadzone_ = this->get_parameter("follow_goal_yaw_deadzone").as_double();
  // --- Edge-of-frame turn ---
  image_width_ = this->get_parameter("image_width").as_double();
  edge_margin_ratio_ = this->get_parameter("edge_margin_ratio").as_double();
  // --- Stationary-target switch ---
  static_target_timeout_sec_ = this->get_parameter("static_target_timeout_sec").as_double();
  static_target_move_thr_ = this->get_parameter("static_target_move_thr").as_double();
  static_switch_depth_diff_thr_ = this->get_parameter("static_switch_depth_diff_thr").as_double();
  static_switch_activity_window_sec_ = this->get_parameter("static_switch_activity_window_sec").as_double();
  // --- Prediction ---
  predict_window_sec_ = this->get_parameter("predict_window_sec").as_double();
  predict_lead_sec_ = this->get_parameter("predict_lead_sec").as_double();
  predict_max_dist_ = this->get_parameter("predict_max_dist").as_double();
  predict_stale_sec_ = this->get_parameter("predict_stale_sec").as_double();
  // --- IDLE search & observe ---
  idle_search_start_timeout_sec_ = this->get_parameter("idle_search_start_timeout_sec").as_double();
  idle_search_total_timeout_sec_ = this->get_parameter("idle_search_total_timeout_sec").as_double();
  idle_observe_duration_sec_ = this->get_parameter("idle_observe_duration_sec").as_double();
  observe_cooldown_sec_ = this->get_parameter("observe_cooldown_sec").as_double();
  spin_radian_ = this->get_parameter("spin_radian").as_double();
  explore_spin_angular_speed_ = this->get_parameter("explore_spin_angular_speed").as_double();
  // --- LOST recovery / belief search / relock ---
  tracking_to_lost_timeout_sec_ = this->get_parameter("tracking_to_lost_timeout_sec").as_double();
  lost_to_idle_timeout_sec_ = this->get_parameter("lost_to_idle_timeout_sec").as_double();
  buzzer_min_interval_sec_ = this->get_parameter("buzzer_min_interval_sec").as_double();
  belief_search_timeout_sec_ = this->get_parameter("belief_search_timeout_sec").as_double();
  belief_search_max_rounds_ = this->get_parameter("belief_search_max_rounds").as_int();
  relock_dist_phase1_ = this->get_parameter("relock_dist_phase1").as_double();
  relock_dist_phase2_ = this->get_parameter("relock_dist_phase2").as_double();
  relock_dist_phase3_ = this->get_parameter("relock_dist_phase3").as_double();
  relock_dist_phase1_sec_ = this->get_parameter("relock_dist_phase1_sec").as_double();
  relock_dist_phase2_sec_ = this->get_parameter("relock_dist_phase2_sec").as_double();
  // --- Misc ---
  costmap_free_cost_thr_ = this->get_parameter("costmap_free_cost_thr").as_double();
  costmap_free_search_radius_m_ = this->get_parameter("costmap_free_search_radius_m").as_double();

  RCLCPP_INFO(this->get_logger(),
    "PersonFollowingNode starting"
    // --- Topics & Frames ---
    "\n          detect_result_topic_name: %s"
    "\n              goal_pose_topic_name: %s"
    "\n                      global_frame: %s"
    "\n                       robot_frame: %s"
    "\n             followed_target_topic: %s"
    "\n        followed_target_pose_topic: %s"
    "\n                     cmd_vel_topic: %s"
    "\n                     costmap_topic: %s"
    // --- Target filtering ---
    "\n           target_filter_width_thr: %.2f"
    "\n          target_filter_height_thr: %.2f"
    "\n         target_filter_range_x_min: %.2f"
    "\n         target_filter_range_x_max: %.2f"
    "\n         target_filter_range_y_min: %.2f"
    "\n         target_filter_range_y_max: %.2f"
    "\n      target_filter_confidence_thr: %.2f"
    // --- Follow distance band ---
    "\n               follow_distance_min: %.2f"
    "\n               follow_distance_max: %.2f"
    "\n                 follow_hysteresis: %.2f"
    "\n         follow_goal_dist_deadzone: %.2f"
    "\n          follow_goal_yaw_deadzone: %.4f"
    // --- Stationary-target switch ---
    "\n         static_target_timeout_sec: %.2f"
    "\n            static_target_move_thr: %.2f"
    "\n      static_switch_depth_diff_thr: %.2f"
    "\n static_switch_activity_window_sec: %.2f"
    // --- Prediction ---
    "\n                predict_window_sec: %.2f"
    "\n                  predict_lead_sec: %.2f"
    "\n                  predict_max_dist: %.2f"
    "\n                 predict_stale_sec: %.2f"
    // --- IDLE search & observe ---
    "\n     idle_search_start_timeout_sec: %.2f"
    "\n     idle_search_total_timeout_sec: %.1f"
    "\n         idle_observe_duration_sec: %.1f"
    "\n              observe_cooldown_sec: %.2f"
    "\n                       spin_radian: %.2f"
    "\n        explore_spin_angular_speed: %.2f"
    // --- LOST recovery ---
    "\n      tracking_to_lost_timeout_sec: %.2f"
    "\n          lost_to_idle_timeout_sec: %.2f"
    // --- Costmap ---
    "\n             costmap_free_cost_thr: %.2f"
    "\n      costmap_free_search_radius_m: %.2f",
    // --- Topics & Frames ---
    detect_result_topic_name_.c_str(),
    goal_pose_topic_name_.c_str(),
    global_frame_.c_str(),
    robot_frame_.c_str(),
    followed_target_topic_.c_str(),
    followed_target_pose_topic_.c_str(),
    cmd_vel_topic_.c_str(),
    costmap_topic_.c_str(),
    // --- Target filtering ---
    target_filter_width_thr_,
    target_filter_height_thr_,
    target_filter_range_x_min_,
    target_filter_range_x_max_,
    target_filter_range_y_min_,
    target_filter_range_y_max_,
    target_filter_confidence_thr_,
    // --- Follow distance band ---
    follow_distance_min_,
    follow_distance_max_,
    follow_hysteresis_,
    follow_goal_dist_deadzone_,
    follow_goal_yaw_deadzone_,
    // --- Stationary-target switch ---
    static_target_timeout_sec_,
    static_target_move_thr_,
    static_switch_depth_diff_thr_,
    static_switch_activity_window_sec_,
    // --- Prediction ---
    predict_window_sec_,
    predict_lead_sec_,
    predict_max_dist_,
    predict_stale_sec_,
    // --- IDLE search & observe ---
    idle_search_start_timeout_sec_,
    idle_search_total_timeout_sec_,
    idle_observe_duration_sec_,
    observe_cooldown_sec_,
    spin_radian_,
    explore_spin_angular_speed_,
    // --- LOST recovery ---
    tracking_to_lost_timeout_sec_,
    lost_to_idle_timeout_sec_,
    // --- Costmap ---
    costmap_free_cost_thr_,
    costmap_free_search_radius_m_);

  // Publisher for cmd_vel used during in-place spin maneuvers.
  // Topic is now parametrized (was hardcoded "/cmd_vel") so spin can be routed
  // through the Nav2 velocity smoother if desired (rev.4 architecture fix 1).
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

  // Follow status publisher on "tros_tracking_status" (std_msgs/String, latched via
  // transient_local) so late subscribers immediately learn the current state.
  // Mirrors frontier_exploration's tros_tracking_status pattern.
  status_pub_ = this->create_publisher<std_msgs::msg::String>(
    status_topic_, rclcpp::QoS(10).transient_local());

  // 蜂鸣器状态提示 publisher：仅 TRACKING->LOST 转换时发 pattern 1（1 短声），
  // 由 originbot_base 订阅 /buzzer_pattern 解码发声；进入 TRACKING 不响。
  buzzer_pattern_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
    buzzer_pattern_topic_, 10);

  // Followed-target info publisher (ai_msgs/PerceptionTargets, latched).
  followed_target_pub_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
    followed_target_topic_, rclcpp::QoS(10).transient_local());

  // Followed-target pose publisher (geometry_msgs/PoseStamped, map frame).
  // Published every frame while TRACKING so external consumers/visualizers can
  // track the followed target's position. Not latched — only valid while
  // tracking; absence of messages signals not-tracking.
  followed_target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    followed_target_pose_topic_, 10);
  std_msgs::msg::String msg;
  msg.data = "follow status: DISABLED";
  status_pub_->publish(msg);

  // Predicted target trajectory (nav_msgs/Path, map frame, latched) for RViz
  // visualization of the prediction quality.
  predict_traj_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    predict_traj_topic_, rclcpp::QoS(10).transient_local());

  // Global costmap subscription for free-cell checks when placing the nav goal.
  global_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    costmap_topic_, rclcpp::QoS(1),
    std::bind(&PersonFollowingNode::globalCostmapCallback, this, _1));

  // Subscribe to AI detection results — drives the main tracking state machine
  detect_result_sub_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
    detect_result_topic_name_,
    1,
    std::bind(&PersonFollowingNode::detectResultCallback, this, _1));

  // Publisher for tracking goal poses (used by external visualization/debugging)
  goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    goal_pose_topic_name_, 1);

  // Publisher for enabling/disabling blind zone observing in the exploration planner
  enable_blind_zone_observing_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "enable_blind_zone_observing", 1);

  // /enable_follow service (rev.4 G9): gates all tracking behavior. Until
  // enabled, the node ignores detections and never spins.
  enable_follow_srv_ = this->create_service<std_srvs::srv::SetBool>(
    "enable_follow",
    std::bind(&PersonFollowingNode::enableFollowCallback, this, _1, _2));

  last_goal_send_time_ = this->now();

  // NavigateToPose action client for sending goals to Nav2
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  // (goal-reachability validation moved to isGoalReachable's costmap cell-free
  // check; no Nav2 ComputePathToPose client needed anymore.)
}

PersonFollowingNode::~PersonFollowingNode() {
  setFollowStatus(FollowStatus::DISABLED);
  // Cancel all pending navigation goals on shutdown to prevent orphaned actions.
  nav_client_->async_cancel_all_goals();
}

void PersonFollowingNode::detectResultCallback(
  const ai_msgs::msg::PerceptionTargets::SharedPtr msg)
{
  RCLCPP_INFO_ONCE(this->get_logger(), "Received detect result.");

  // Cache the latest detection frame for publishFollowedTarget.
  latest_detection_ = *msg;

  // Publish the followed target info for every received detection frame,
  // independent of follow_status_ changes.
  publishFollowedTarget();

  // G9: /enable_follow gates all tracking behavior. Until enabled, ignore
  // detections entirely (no spin search, no tracking). This makes the service
  // the single entry point for following, matching the spec.
  if (!follow_enabled_) {
    return;
  }

  std::vector<PersonTarget> persons;

  for (const auto & obj : msg->targets) {
    if (obj.type == "person" && obj.attributes.size() > 0) {
      bool has_x_cm = false;
      bool has_y_cm = false;
      bool has_height_cm = false;
      bool has_width_cm = false;
      bool has_y_offset = true;
      float height_m = 0.0;
      float width_m = 0.0;

      geometry_msgs::msg::PoseStamped pose_camera;
      pose_camera.header.stamp = msg->header.stamp;
      pose_camera.header.frame_id = "camera_link";

      for (const auto & attr : obj.attributes) {
        if (attr.type == "x_cm") {
          pose_camera.pose.position.x = attr.value / 100.0;
          has_x_cm = true;
        } else if (attr.type == "y_cm") {
          pose_camera.pose.position.y = attr.value / 100.0;
          has_y_cm = true;
        } else if (attr.type == "height_cm") {
          height_m = attr.value / 100.0;
          has_height_cm = true;
        } else if (attr.type == "width_cm") {
          width_m = attr.value / 100.0;
          has_width_cm = true;
        } else if (attr.type == "y_offset") {
          has_y_offset = true;
        }
      }

      if (has_x_cm && has_y_cm && has_height_cm && has_width_cm && has_y_offset) {
        if (height_m < target_filter_height_thr_) {
          if (track_state_ != TrackState::TRACKING) {
            RCLCPP_INFO(this->get_logger(), "Invalid person height: %.2f, less than thr: %.2f m.",
              height_m, target_filter_height_thr_);
          }
          continue;
        }
        if (pose_camera.pose.position.x < target_filter_range_x_min_ || pose_camera.pose.position.x > target_filter_range_x_max_ ||
            pose_camera.pose.position.y < target_filter_range_y_min_ || pose_camera.pose.position.y > target_filter_range_y_max_) {
          RCLCPP_INFO(this->get_logger(), "Out of range x: %.2f [%.2f, %.2f], y: %.2f [%.2f, %.2f].",
            pose_camera.pose.position.x, target_filter_range_x_min_, target_filter_range_x_max_,
            pose_camera.pose.position.y, target_filter_range_y_min_, target_filter_range_y_max_);
          continue;
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Parsed a person at cam frame pose x: %.2f, y: %.2f, w: %.2f, h: %.2f.",
          pose_camera.pose.position.x, pose_camera.pose.position.y, width_m, height_m);
        // Extract person ROI confidence and bbox center for multi-factor selection.
        float person_conf = 0.0f;
        float person_bbox_center_x = 0.0f;
        for (const auto & roi : obj.rois) {
          if (roi.type == "person") {
            person_conf = roi.confidence;
            person_bbox_center_x = roi.rect.x_offset + roi.rect.width / 2.0f;
            break;
          }
        }
        persons.push_back({&obj,
          pose_camera.pose.position.x,
          pose_camera.pose.position.y,
          height_m,
          width_m,
          person_conf,
          person_bbox_center_x});
      }
    }
  }

  // Record each detected person's world-frame position for stationary-target
  // detection (A) and activity scoring (candidate B). Prune old samples first.
  {
    geometry_msgs::msg::PoseStamped robot_pose_now = getCurrentPose();
    pruneTargetPosHistory(static_target_timeout_sec_ + static_switch_activity_window_sec_);
    for (const auto & p : persons) {
      geometry_msgs::msg::PoseStamped pc;
      pc.header.stamp = msg->header.stamp;
      pc.header.frame_id = "camera_link";
      pc.pose.position.x = p.x_m;
      pc.pose.position.y = p.y_m;
      auto pw = transformPersonToWorld(robot_pose_now, pc);
      if (!pw.header.frame_id.empty()) {
        target_pos_history_[p.target->track_id].push_back({pw.pose.position, this->now()});
      }
    }
  }

  // State machine: handle LOST timeout -> IDLE transition
  if (track_state_ == TrackState::LOST) {
    // LOST→IDLE timeout is suspended while belief-guided search is active so
    // the search can run to completion (nav to observation points + scans +
    // max_rounds iterations, bounded by belief_search_timeout_sec_). Without
    // this gate the short lost_to_idle_timeout_sec_ would abort an in-flight nav/scan
    // mid-search (observed: nav reached at +0.9s, LOST timeout fired at +1.0s,
    // cutting off the scan). belief_search_.active flips to false when the
    // search ends (timeout / rounds exhausted / no observation point / target
    // recovered), at which point this gate opens and LOST→IDLE proceeds.
    if (!belief_search_.active &&
        (this->now() - tp_lost_).seconds() > lost_to_idle_timeout_sec_) {
      RCLCPP_INFO(this->get_logger(),
        "[LOST-FLOW 13] LOST timeout %.2f sec reached (belief search inactive), LOST -> IDLE",
        lost_to_idle_timeout_sec_);
      belief_search_.active = false;  // end belief-guided search
      publishBeliefSearchPath();  // clear the search path display
      spin_stop_requested_ = true;   // stop any in-flight scan
      setTrackState(TrackState::IDLE);
      tracking_track_id_ = 0;
      // Restart the search-timeout window now that we're (re)entering IDLE,
      // so continue_search's `elapsed < idle_search_total_timeout_sec_` gate allows
      // spinning for the next idle_search_total_timeout_sec_ seconds.
      tp_target_find_start_ = this->now();
      observe_track_id_ = 0;
      publishBlindZoneObserving(true);
      setFollowStatus(FollowStatus::IDLE_SEARCHING);
      // Cancel any nav goal still active from before/within LOST so the IDLE
      // spin search owns /cmd_vel exclusively (no Nav2 controller racing it).
      if (nav_goal_handle_) {
        RCLCPP_INFO(this->get_logger(), "Canceling nav goal on transitioning to IDLE");
        nav_client_->async_cancel_goal(nav_goal_handle_);
        nav_goal_handle_ = nullptr;
        last_nav_goal_pose_ = nullptr;
      }
    }
  }

  if (track_state_ == TrackState::TRACKING) {
    // Try to find the tracked target by track_id
    bool found = false;
    for (const auto & p : persons) {
      if (p.target->track_id == tracking_track_id_) {
        found = true;
        publishGoalPose(*p.target, msg->header.stamp);
        break;
      }
    }
    if (!found) {
      // Transition into LOST: set state, cancel the stale follow nav goal, and
      // start the belief-guided search (prediction-first + belief fallback).
      // Extracted as a lambda so the pending-lost shortcut below reuses it.
      auto enter_lost = [&]() {
        RCLCPP_INFO(this->get_logger(),
          "[LOST-FLOW 2] target id=%lu lost, transitioning TRACKING -> LOST",
          tracking_track_id_);
        setTrackState(TrackState::LOST);
        following_active_ = false;  // reset hysteresis state: next TRACKING re-enters via enter threshold
        tp_lost_ = this->now();
        target_lost_ = false;
        // Note: enable_blind_zone_observing stays false during LOST (it was
        // paused when entering TRACKING). It is only re-enabled on LOST→IDLE
        // transition and on stopFollowing — blind-zone observing runs while
        // truly idle (not while the belief search / nav is in flight).
        setFollowStatus(FollowStatus::LOST);
        // Cancel the active follow nav goal: the tracked person is lost, so the
        // old goal (aimed at where the person was) is stale. This also prevents
        // the IDLE spin search from racing Nav2 for /cmd_vel control later.
        if (nav_goal_handle_) {
          RCLCPP_INFO(this->get_logger(),
            "[LOST-FLOW 3] canceling active nav goal on entering LOST (target id=%lu)",
            tracking_track_id_);
          nav_client_->async_cancel_goal(nav_goal_handle_);
          nav_goal_handle_ = nullptr;
          last_nav_goal_pose_ = nullptr;
        }
        // Proactive search: on entering LOST, start belief-guided search
        // (prediction-first + belief fallback, unified framework).
        startBeliefSearch();
      };
      // Track consecutive lost duration before transitioning to LOST.
      if (!target_lost_) {
        // First frame the target is not found.
        // Shortcut: the pending-lost window's only value is to let an in-flight
        // follow nav goal keep executing through a brief occlusion (the target is
        // expected back next frame). If there is no active nav goal, waiting the
        // full tracking_to_lost_timeout_sec_ is pure idle time with no benefit — go
        // straight to LOST and start the active recovery (belief search).
        // Skip if a goal is pending (just sent, awaiting accept) — that counts
        // as an in-flight goal the pending-lost window should protect.
        if (!nav_goal_handle_ && !goal_pending_) {
          enter_lost();
        } else {
          target_lost_ = true;
          tp_target_lost_ = this->now();
          RCLCPP_INFO(this->get_logger(),
            "[LOST-FLOW 1] target id=%lu not detected this frame, entering pending-lost (wait %.1fs before LOST, in-flight nav continues)",
            tracking_track_id_, tracking_to_lost_timeout_sec_);
          setFollowStatus(FollowStatus::WILL_BE_LOST);  // signal the pending-lost window (distinct from TRACKING)
        }
      } else if ((this->now() - tp_target_lost_).seconds() > tracking_to_lost_timeout_sec_) {
        // Target has been continuously lost beyond threshold -> LOST
        enter_lost();
      }
    } else {
      // Target found again, reset lost flag
      target_lost_ = false;

      // Stationary-target switch: if the tracked A has been (near-)stationary
      // for static_target_timeout_sec_ and a different person B is at a close
      // depth and is itself moving, switch to the most active such B so the
      // robot doesn't sit still following a stationary person.
      double a_move = targetMovementOverWindow(tracking_track_id_, static_target_timeout_sec_);
      if (a_move >= 0.0 && a_move < static_target_move_thr_) {
        const PersonTarget * a_ptr = nullptr;
        for (const auto & p : persons) {
          if (p.target->track_id == tracking_track_id_) { a_ptr = &p; break; }
        }
        if (a_ptr) {
          const PersonTarget * best_b = nullptr;
          double best_b_move = static_target_move_thr_;  // B must move more than this
          for (const auto & p : persons) {
            if (p.target->track_id == tracking_track_id_) continue;
            if (std::fabs(p.x_m - a_ptr->x_m) > static_switch_depth_diff_thr_) continue;
            double mv = targetMovementOverWindow(p.target->track_id, static_switch_activity_window_sec_);
            if (mv > 0.0 && mv > best_b_move) {
              best_b = &p;
              best_b_move = mv;
            }
          }
          if (best_b) {
            RCLCPP_INFO(this->get_logger(),
              "Tracked target id=%lu stationary (move=%.2f < %.2f over %.0fs); "
              "switching to active target id=%lu (move=%.2f)",
              tracking_track_id_, a_move, static_target_move_thr_,
              static_target_timeout_sec_, best_b->target->track_id, best_b_move);
            // Cancel the nav goal aimed at A so a fresh goal for B is sent.
            if (nav_goal_handle_) {
              nav_client_->async_cancel_goal(nav_goal_handle_);
              nav_goal_handle_ = nullptr;
            }
            last_nav_goal_pose_ = nullptr;
            tracking_track_id_ = best_b->target->track_id;
            following_active_ = false;  // reset hysteresis: new target enters via threshold, not residual follow
            // Force a status republish so the new id is emitted: setFollowStatus
            // dedupes, so reset to a non-TRACKING value first.
            follow_status_ = FollowStatus::LOST;
            setFollowStatus(FollowStatus::TRACKING);
            publishGoalPose(*best_b->target, msg->header.stamp);
          }
        }
      }
    }
    return;
  }

  if (track_state_ == TrackState::LOST) {
    // Still in LOST (not timed out yet), try to recover the same target by track_id
    for (const auto & p : persons) {
      if (p.target->track_id == tracking_track_id_) {
        // Distance gate: re-locked target must be within currentRelockDist()
        // (LOST-duration-phased: strict early, lenient late) of the LKP/predicted
        // point. Otherwise the original id was re-detected (e.g. MOT re-assigned)
        // on an unrelated person far away, and locking on would steer the robot
        // off-course. Skip and keep searching.
        geometry_msgs::msg::Point ref_lkp;
        if (getRelockLkp(ref_lkp)) {
          geometry_msgs::msg::PoseStamped pc;
          pc.header.stamp = this->now();
          pc.header.frame_id = "camera_link";
          pc.pose.position.x = p.x_m;
          pc.pose.position.y = p.y_m;
          auto tr_pose = transformPersonToWorld(getCurrentPose(), pc);
          double d = std::hypot(tr_pose.pose.position.x - ref_lkp.x,
                                tr_pose.pose.position.y - ref_lkp.y);
          const double relock_thr = currentRelockDist();
          if (d > relock_thr) {
            RCLCPP_INFO(this->get_logger(),
              "[LOST-FLOW 7] original id=%lu re-detected at %.2f m from LKP (>%.2f m, t=%.1fs), skip (likely unrelated)",
              tracking_track_id_, d, relock_thr, (this->now() - tp_lost_).seconds());
            continue;
          }
        }
        RCLCPP_INFO(this->get_logger(),
          "[LOST-FLOW 7] recovered original target id=%lu by track_id, LOST -> TRACKING",
          tracking_track_id_);
        belief_search_.active = false;  // interrupt belief-guided search
        spin_stop_requested_ = true;   // stop any in-flight scan
        setTrackState(TrackState::TRACKING);
        following_active_ = false;  // reset hysteresis: re-acquired target enters via threshold
        target_lost_ = false;
        tp_target_find_start_ = this->now();
        publishBlindZoneObserving(false);
        setFollowStatus(FollowStatus::TRACKING);
        // TODO: Decrease lifelong of dynamic obstacles
        publishGoalPose(*p.target, msg->header.stamp);
        return;
      }
    }
    // Original target not found: accept any newly detected person that meets the
    // follow conditions (valid + moving) so the robot doesn't sit idle in LOST
    // when a perfectly followable target is present. This skips the IDLE observe
    // window for faster re-lock. Requires "moving" to avoid locking onto a
    // stationary person. (Improvement B)
    {
      const PersonTarget * new_target = nullptr;
      geometry_msgs::msg::Point ref_lkp;
      const bool have_lkp = getRelockLkp(ref_lkp);
      for (const auto & p : persons) {
        if (p.target->track_id == tracking_track_id_) continue;  // already checked, absent
        // Confidence + size thresholds (same filter as the IDLE-state selection).
        float conf = 0.0f;
        for (const auto & roi : p.target->rois) {
          if (roi.type == "person") { conf = roi.confidence; break; }
        }
        if (conf < target_filter_confidence_thr_) continue;
        if (!(p.width_m >= target_filter_width_thr_ && p.height_m >= target_filter_height_thr_)) continue;
        // Must be moving (same definition as IDLE-state selection) to avoid
        // locking onto a stationary person during LOST.
        double move = targetMovementOverWindow(p.target->track_id, static_switch_activity_window_sec_);
        if (!(move > static_target_move_thr_)) continue;
        // Distance gate: re-locked target must be within currentRelockDist()
        // (LOST-duration-phased: strict early, lenient late) of the LKP/predicted
        // point. Far-away moving persons (likely unrelated) are not the original
        // target.
        if (have_lkp) {
          geometry_msgs::msg::PoseStamped pc;
          pc.header.stamp = this->now();
          pc.header.frame_id = "camera_link";
          pc.pose.position.x = p.x_m;
          pc.pose.position.y = p.y_m;
          auto tr_pose = transformPersonToWorld(getCurrentPose(), pc);
          double d = std::hypot(tr_pose.pose.position.x - ref_lkp.x,
                                tr_pose.pose.position.y - ref_lkp.y);
          const double relock_thr = currentRelockDist();
          if (d > relock_thr) {
            RCLCPP_DEBUG(this->get_logger(),
              "[LOST-FLOW 8] candidate id=%lu at %.2f m from LKP (>%.2f m, t=%.1fs), skip (likely unrelated)",
              p.target->track_id, d, relock_thr, (this->now() - tp_lost_).seconds());
            continue;
          }
        }
        // Pick the best such moving target by multi-factor score.
        if (!new_target || computeTargetScore(p) > computeTargetScore(*new_target)) {
          new_target = &p;
        }
      }
      if (new_target) {
        // 新目标相对 LKP 的距离 + 当前 timeline 阈值 + LOST 持续时间（一行可对照锁定合理性）。
        if (have_lkp) {
          geometry_msgs::msg::PoseStamped pc;
          pc.header.stamp = this->now();
          pc.header.frame_id = "camera_link";
          pc.pose.position.x = new_target->x_m;
          pc.pose.position.y = new_target->y_m;
          auto tr_pose = transformPersonToWorld(getCurrentPose(), pc);
          double d_new = std::hypot(tr_pose.pose.position.x - ref_lkp.x,
                                    tr_pose.pose.position.y - ref_lkp.y);
          const double t_lost = (this->now() - tp_lost_).seconds();
          const double relock_thr = currentRelockDist();
          RCLCPP_INFO(this->get_logger(),
            "[LOST-FLOW 8] LOST: switching from id=%lu to new moving target id=%lu, LOST -> TRACKING (d(new,LKP)=%.2f m, thr=%.2f m, t=%.1fs)",
            tracking_track_id_, new_target->target->track_id, d_new, relock_thr, t_lost);
        } else {
          RCLCPP_INFO(this->get_logger(),
            "[LOST-FLOW 8] LOST: switching from id=%lu to new moving target id=%lu, LOST -> TRACKING (no LKP, distance N/A)",
            tracking_track_id_, new_target->target->track_id);
        }
        belief_search_.active = false;  // interrupt belief-guided search
        spin_stop_requested_ = true;   // stop any in-flight scan
        // Cancel the in-flight nav goal (if any) so a fresh goal for the new
        // target is sent (publishGoalPose's re-nav branch will cancel + resend).
        if (nav_goal_handle_) {
          nav_client_->async_cancel_goal(nav_goal_handle_);
          nav_goal_handle_ = nullptr;
        }
        last_nav_goal_pose_ = nullptr;
        tracking_track_id_ = new_target->target->track_id;
        setTrackState(TrackState::TRACKING);
        following_active_ = false;  // reset hysteresis: new target enters via threshold
        target_lost_ = false;
        tp_target_find_start_ = this->now();
        publishBlindZoneObserving(false);
        // Force a republish with the new id (bypass the setFollowStatus dedupe).
        follow_status_ = FollowStatus::LOST;
        setFollowStatus(FollowStatus::TRACKING);
        publishGoalPose(*new_target->target, msg->header.stamp);
        return;
      }
    }
    // Neither original id nor a new moving target found this frame. If a belief
    // search is active and its in-place scan just finished (scan_completed set
    // by the scan thread), advance it to the next round here — the scan publishes
    // cmd_vel, not a nav goal, so no resultCallback fires to drive continuation.
    // Without this poll the search would idle until belief_search_timeout_sec_.
    // With the LOST-timeout-suspended-while-active change, this poll is what lets
    // the search actually run its max_rounds iterations.
    if (belief_search_.active && belief_search_.scan_completed) {
      continueBeliefSearch();
    }
    return;
  }

  // IDLE state: select the nearest person to track.
  // Filter persons by width, height, and confidence thresholds first.
  std::vector<PersonTarget> valid_persons;
  for (const auto & p : persons) {
    // Extract confidence from rois where type == "person"
    float conf = 0.0f;
    for (const auto & roi : p.target->rois) {
      if (roi.type == "person") {
        conf = roi.confidence;
        break;
      }
    }
    if (conf < target_filter_confidence_thr_) {
      RCLCPP_DEBUG(this->get_logger(),
        "Person confidence %.2f < thr %.2f, skipping.", conf, target_filter_confidence_thr_);
      continue;
    }
    if (p.width_m >= target_filter_width_thr_ && p.height_m >= target_filter_height_thr_) {
      RCLCPP_INFO(this->get_logger(),
        "Valid person id=%lu, x=%.2f, y=%.2f, w=%.2f, h=%.2f, conf=%.2f",
        p.target->track_id, p.x_m, p.y_m, p.width_m, p.height_m, conf);
      valid_persons.push_back(p);
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Person width %.2f < thr %.2f or height %.2f < thr %.2f, skipping.",
        p.width_m, target_filter_width_thr_, p.height_m, target_filter_height_thr_);
    }
  }

  // Helper: continue the spin search (used when no target to follow/observe).
  auto continue_search = [&]() {
      setFollowStatus(FollowStatus::IDLE_SEARCHING);
      // Any new observation target is invalidated when we go back to searching.
      observe_track_id_ = 0;
      tracking_track_id_ = 0;
      double elapsed = (this->now() - tp_target_find_start_).seconds();
      if (elapsed < idle_search_total_timeout_sec_) {
        if ((this->now() - tp_lost_).seconds() > idle_search_start_timeout_sec_) {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
            "Spin to detect person (elapsed=%.1f/%.1f s).",
            elapsed, idle_search_total_timeout_sec_);
          // Run SpinInPlaceWCmdVel asynchronously so the detection callback
          // keeps firing during the spin — otherwise valid persons seen while
          // spinning would be missed (the spin blocks the executor). The spin
          // is stoppable via spin_stop_requested_ once a target is found.
          if (!spin_active_.load()) {
            spin_active_ = true;
            spin_stop_requested_ = false;
            // Turn toward the side where the target disappeared (recorded while
            // tracking). spin_radian_ magnitude with last_target_direction_ sign
            // (+1 left / -1 right).
            float dir = last_target_direction_;
            spin_thread_ = std::make_shared<std::thread>([this, dir]() {
              SpinInPlaceWCmdVel(spin_radian_ * dir);
              spin_active_ = false;
            });
            spin_thread_->detach();
          }
        }
      } else {
        RCLCPP_DEBUG(this->get_logger(), "Target find timeout (%.1f s), stop spinning.",
          idle_search_total_timeout_sec_);
      }
    };

  if (valid_persons.empty()) {
    // No valid target this frame. If we were observing a candidate (within its
    // observe window), keep waiting instead of spinning — the candidate may
    // have just dropped a frame (perception miss) and reappear. Only resume the
    // spin search once the observation window has elapsed.
    if (observe_track_id_ != 0 &&
        (this->now() - tp_observe_start_).seconds() < idle_observe_duration_sec_) {
      tracking_track_id_ = observe_track_id_;
      setFollowStatus(FollowStatus::IDLE_OBSERVING);
      RCLCPP_DEBUG(this->get_logger(),
        "No valid person this frame; still observing id=%lu (%.1fs/%.1fs), stay still",
        observe_track_id_, (this->now() - tp_observe_start_).seconds(), idle_observe_duration_sec_);
      return;
    }
    continue_search();
    return;
  }

  // Keep only moving targets (avoid locking onto a stationary person). A
  // target is "moving" if its world-frame movement over static_switch_activity_window_sec_
  // exceeds static_target_move_thr_ (same definition as the stationary-switch and
  // edge-turn logic). Insufficient samples (targetMovementOverWindow < 0) don't
  // qualify — newly appeared targets need to accumulate motion before being picked.
  std::vector<PersonTarget> moving_persons;
  std::map<uint64_t, double> move_by_id;  // track_id -> movement (m) for logging
  for (const auto & p : valid_persons) {
    double move = targetMovementOverWindow(p.target->track_id, static_switch_activity_window_sec_);
    move_by_id[p.target->track_id] = move;
    if (move > static_target_move_thr_) {
      moving_persons.push_back(p);
    }
  }

  if (moving_persons.empty()) {
    // Cooldown after a recent observe-timeout: a valid-but-stationary target
    // timed out without moving. For observe_cooldown_sec_ skip re-observing ANY
    // stationary valid target and let the IDLE spin actually turn to find a
    // moving one — otherwise the same stationary id loops observe→timeout→spin
    // every idle_observe_duration_sec_. (Global cooldown: may skip observing a
    // newly-appeared stationary id during the window — acceptable, since a truly
    // followable target will register move>thr and go through the moving branch,
    // not this one.)
    if (tp_observe_timeout_.nanoseconds() != 0 &&
        (this->now() - tp_observe_timeout_).seconds() < observe_cooldown_sec_) {
      observe_track_id_ = 0;
      continue_search();
      return;
    }
    // A valid (distance-condition) but not-yet-moving target is present. Pause
    // spinning and observe the nearest one for up to idle_observe_duration_sec_: if it
    // moves during the window, follow it; otherwise resume searching.
    const auto & nearest_valid = *std::min_element(valid_persons.begin(), valid_persons.end(),
        [](const PersonTarget & a, const PersonTarget & b) { return a.x_m < b.x_m; });
    uint64_t cand_id = nearest_valid.target->track_id;

    // If the candidate disappeared or changed, start a fresh observation.
    bool candidate_present = false;
    for (const auto & p : persons) {
      if (p.target->track_id == cand_id) { candidate_present = true; break; }
    }
    if (!candidate_present || observe_track_id_ != cand_id) {
      observe_track_id_ = cand_id;
      tp_observe_start_ = this->now();
      RCLCPP_INFO(this->get_logger(),
        "Observing candidate target id=%lu for up to %.1fs before follow/search",
        cand_id, idle_observe_duration_sec_);
      // Stop any in-flight async spin so the robot stays still to observe.
      spin_stop_requested_ = true;
      // Pause spinning (do not call SpinInPlaceWCmdVel) — just stay still and
      // keep accumulating motion samples. Report IDLE_OBSERVING status.
      tracking_track_id_ = observe_track_id_;
      setFollowStatus(FollowStatus::IDLE_OBSERVING);
      return;
    }

    // Still observing the same candidate. If it has started moving, follow it.
    double move = targetMovementOverWindow(cand_id, static_switch_activity_window_sec_);
    if (move > static_target_move_thr_) {
      // Fall through to the follow block below using this candidate as nearest.
      // (Re-run the moving selection so the same code path picks it.)
      moving_persons.push_back(nearest_valid);
    } else if ((this->now() - tp_observe_start_).seconds() < idle_observe_duration_sec_) {
      // Within the observation window and still not moving — keep observing.
      tracking_track_id_ = observe_track_id_;
      setFollowStatus(FollowStatus::IDLE_OBSERVING);
      return;
    } else {
      // Observation timed out with no movement — resume searching. Record the
      // timeout so the cooldown gate above skips re-observing (any stationary
      // valid target) for observe_cooldown_sec_, letting the spin actually turn.
      RCLCPP_INFO(this->get_logger(),
        "Candidate id=%lu did not move within %.1fs, resuming search (cooldown %.1fs)",
        cand_id, idle_observe_duration_sec_, observe_cooldown_sec_);
      tp_observe_timeout_ = this->now();
      observe_track_id_ = 0;
      continue_search();
      return;
    }
  } else {
    // A moving target appeared — cancel any pending observation and follow it.
    observe_track_id_ = 0;
  }

  // Stricter confidence gate for IDLE target selection: only moving candidates
  // with conf >= select_min_confidence_ are eligible to be picked as the follow
  // target. Lower-quality ones (conf in [target_filter_confidence_thr_, select_min_confidence_))
  // are kept out — keep spinning to find a higher-quality target.
  {
    std::vector<PersonTarget> strict_moving;
    strict_moving.reserve(moving_persons.size());
    for (const auto & p : moving_persons) {
      if (p.confidence >= select_min_confidence_) {
        strict_moving.push_back(p);
      }
    }
    if (strict_moving.empty()) {
      RCLCPP_INFO(this->get_logger(),
        "IDLE: %zu moving candidate(s) but none conf >= %.2f (select_min_confidence), keep searching",
        moving_persons.size(), select_min_confidence_);
      observe_track_id_ = 0;
      continue_search();
      return;
    }
    moving_persons = std::move(strict_moving);
  }

  // Select best moving target by multi-factor score (distance + confidence + center).
  const auto & best = *std::max_element(moving_persons.begin(), moving_persons.end(),
    [this](const PersonTarget & a, const PersonTarget & b) {
      return computeTargetScore(a) < computeTargetScore(b);
    });

  // Log why this target was selected: its score vs all valid candidates.
  {
    std::string cand_str;
    for (const auto & p : valid_persons) {
      if (!cand_str.empty()) cand_str += ", ";
      auto it = move_by_id.find(p.target->track_id);
      double mv = (it != move_by_id.end()) ? it->second : -1.0;
      float sc = computeTargetScore(p);
      cand_str += "id=" + std::to_string(p.target->track_id) +
                  "(x=" + std::to_string(p.x_m) + ",conf=" + std::to_string(p.confidence) +
                  ",cx=" + std::to_string(p.bbox_center_x) + ",score=" + std::to_string(sc) +
                  ",move=" + std::to_string(mv) + ")";
    }
    float best_score = computeTargetScore(best);
    double best_move = -1.0;
    auto bit = move_by_id.find(best.target->track_id);
    if (bit != move_by_id.end()) best_move = bit->second;
    RCLCPP_INFO(this->get_logger(),
      "Selected best target id=%lu (score=%.3f, x=%.2f, conf=%.2f, cx=%.0f, move=%.3f over %.1fs)."
      "\nCandidates: %s",
      best.target->track_id, best_score, best.x_m, best.confidence, best.bbox_center_x,
      best_move, static_switch_activity_window_sec_, cand_str.c_str());
  }

  // Transition IDLE -> TRACKING: acquire new target and disable blind zone observing
  spin_stop_requested_ = true;  // stop async spin before issuing nav goals
  tracking_track_id_ = best.target->track_id;
  setTrackState(TrackState::TRACKING);
  following_active_ = false;  // reset hysteresis: new target enters via threshold
  tp_target_find_start_ = this->now();
  publishBlindZoneObserving(false);
  setFollowStatus(FollowStatus::TRACKING);
  RCLCPP_INFO(this->get_logger(), "Selected target (id=%lu) for tracking.", tracking_track_id_);
  publishGoalPose(*best.target, msg->header.stamp);
}

void PersonFollowingNode::enableFollowCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) {
    if (follow_enabled_) {
      response->success = true;
      response->message = "Follow already enabled";
      return;
    }
    startFollowing();
    response->success = true;
    response->message = "Follow enabled";
  } else {
    if (!follow_enabled_) {
      response->success = true;
      response->message = "Follow already disabled";
      return;
    }
    stopFollowing();
    response->success = true;
    response->message = "Follow disabled";
  }
}

void PersonFollowingNode::startFollowing()
{
  // G12 (partial): cancel any in-flight follow goal so follow owns Nav2 from
  // the start. async_cancel_all_goals cancels every goal sent by THIS action
  // client (goals we sent whose handle may not have been accepted yet are
  // also covered). It does NOT cancel goals sent by other clients (e.g. RViz
  // /goal_pose uses a separate client); those are unaffected. Mirrors
  // stopFollowing()'s cancel approach so enter/leave-follow are symmetric.
  nav_client_->async_cancel_all_goals();
  nav_goal_handle_ = nullptr;
  last_nav_goal_pose_ = nullptr;

  follow_enabled_ = true;
  setTrackState(TrackState::IDLE);
  tracking_track_id_ = 0;
  following_active_ = false;  // start clean: first target enters via threshold
  target_lost_ = false;
  target_pos_history_.clear();
  observe_track_id_ = 0;
  belief_search_.active = false;
  tp_target_find_start_ = this->now();
  tp_lost_ = this->now();

  // Arm the goal re-send rate limiter.
  goal_send_armed_ = true;
  last_goal_send_time_ = this->now();

  setFollowStatus(FollowStatus::IDLE_SEARCHING);
  RCLCPP_INFO(this->get_logger(), "Follow enabled");
}

void PersonFollowingNode::stopFollowing()
{
  // Cancel all in-flight follow goals. Uses async_cancel_all_goals (matching
  // startFollowing) instead of async_cancel_goal(nav_goal_handle_) so that
  // goals which were just async_send_goal'd but haven't yet been accepted by
  // the server (nav_goal_handle_ still nullptr in goalResponseCallback) are
  // also cancelled — otherwise such a goal slips through and runs to
  // completion after disable.
  nav_client_->async_cancel_all_goals();
  nav_goal_handle_ = nullptr;
  last_nav_goal_pose_ = nullptr;
  RCLCPP_INFO(this->get_logger(), "Cancelling all nav goals");

  // Stop any in-progress spin and release Nav2 control.
  spin_stop_requested_ = true;
  if (cmd_vel_pub_) {
    geometry_msgs::msg::Twist stop_twist;
    cmd_vel_pub_->publish(stop_twist);
  }
  publishBlindZoneObserving(true);

  follow_enabled_ = false;
  setTrackState(TrackState::IDLE);
  tracking_track_id_ = 0;
  following_active_ = false;  // defensive: reset hysteresis state on disable
  target_lost_ = false;
  target_pos_history_.clear();
  observe_track_id_ = 0;
  belief_search_.active = false;
  setFollowStatus(FollowStatus::DISABLED);
  RCLCPP_INFO(this->get_logger(), "Follow disabled, control returned to Nav2");
}

void PersonFollowingNode::publishGoalPose(
  const ai_msgs::msg::Target & target,
  const builtin_interfaces::msg::Time & stamp)
{
  geometry_msgs::msg::PoseStamped pose_camera;
  pose_camera.header.stamp = stamp;
  pose_camera.header.frame_id = "camera_link";
  float width = 0;
  float height = 0;
  float confidence = 0;
  for (const auto & roi : target.rois) {
    RCLCPP_DEBUG(this->get_logger(), "ROI type: %s, confidence: %.2f",
      roi.type.c_str(), roi.confidence);
    if (roi.type == "person") {
      confidence = roi.confidence;
      break;
    }
  }
          
  for (const auto & attr : target.attributes) {
    if (attr.type == "x_cm") {
      pose_camera.pose.position.x = attr.value / 100.0;
    } else if (attr.type == "y_cm") {
      pose_camera.pose.position.y = attr.value / 100.0;
    } else if (attr.type == "width_cm") {
      width = attr.value / 100.0;
    } else if (attr.type == "height_cm") {
      height = attr.value / 100.0;
    }
  }

  // Get current robot pose in global frame (for distance/yaw comparisons below)
  geometry_msgs::msg::PoseStamped current_pose = getCurrentPose();

  // Transform camera-relative target pose to global frame
  auto transformed_target_pose = pose_camera;
  if (current_pose.header.frame_id != transformed_target_pose.header.frame_id) {
    if (!nav2_util::transformPoseInTargetFrame(
        pose_camera, transformed_target_pose, tf_buffer_, current_pose.header.frame_id))
    {
      RCLCPP_ERROR(this->get_logger(),
        "Could not transform %s to %s at line: %d",
        transformed_target_pose.header.frame_id.c_str(),
        current_pose.header.frame_id.c_str(), __LINE__);
      return;
    }
    transformed_target_pose.header.frame_id = current_pose.header.frame_id;
  }

  // Publish the followed target's map-frame pose every frame while tracking.
  publishFollowedTargetPose(transformed_target_pose);

  // Person distance in the camera frame (x = forward, in meters).
  double person_dist = pose_camera.pose.position.x;
  // Cache it so setFollowStatus (deduped, fires on the status-change frame
  // before this point) can append :dist= consistently with the per-frame reports.
  last_target_dist_ = person_dist;

  // Map-frame robot↔person planar Euclidean distance — the same distance the
  // follow-distance band logic below uses (vs person_dist, the camera-frame
  // forward depth). Computed here (ahead of the per-frame distance report and
  // the control decisions) so both publish the same authoritative value, and
  // cached so setFollowStatus can append it on the status-change frame.
  double dx_person = transformed_target_pose.pose.position.x - current_pose.pose.position.x;
  double dy_person = transformed_target_pose.pose.position.y - current_pose.pose.position.y;
  double person_map_dist = std::hypot(dx_person, dy_person);
  last_target_map_dist_ = person_map_dist;

  // Per-frame distance report (un-throttled): publish the map-frame robot↔person
  // Euclidean distance to the followed target on the status topic (explore/status,
  // std_msgs/String) every frame while following — bypasses setFollowStatus's
  // dedup so consumers get a live distance update at detection rate. Emitted
  // before the min-distance short-circuit so too-close frames are reported too.
  // String format mirrors setFollowStatus: "FollowStatus <STATE>:id=N:dist=X.XX"
  // (the :dist= suffix is additive, so existing parsers that key on the status
  // name / :id= are unaffected).
  if (status_pub_) {
    std::string dist_str = followStatusToStr(follow_status_);
    dist_str += ":id=" + std::to_string(tracking_track_id_);
    char dist_buf[16];
    std::snprintf(dist_buf, sizeof(dist_buf), ":dist=%.2f", person_map_dist);
    dist_str += dist_buf;
    std_msgs::msg::String status_msg;
    status_msg.data = dist_str;
    status_pub_->publish(status_msg);
    RCLCPP_INFO(this->get_logger(), "follow status: %s", dist_str.c_str());
  }

  // Record the target's lateral direction (camera-frame y: + left / - right)
  // so the search spin can turn toward the side where the target disappeared.
  if (std::abs(pose_camera.pose.position.y) > 1e-6) {
    last_target_direction_ = (pose_camera.pose.position.y > 0.0f) ? 1.0f : -1.0f;
  }
  // G11: hard min-distance stop. If the person is closer than follow_min_safe_distance,
  // stop sending ALL goals (including orientation updates) and let Nav2 coast
  // to a stop. This is a safety guard beyond the approach-distance cap.
  if (person_dist < follow_min_safe_distance_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Person too close (%.2f m < %.2f), withholding goal.", person_dist, follow_min_safe_distance_);
    setFollowStatus(FollowStatus::TRACKING_TOO_CLOSE);
    return;
  }

  // Map-frame robot↔person Euclidean distance (person_map_dist) is computed
  // above, ahead of the per-frame distance report, so both publish the same
  // authoritative value.

  if (follow_status_ == FollowStatus::TRACKING) {
    // Person detection-box pixel coords (for edge analysis).
    int32_t bx = 0, by = 0, bw = 0, bh = 0;
    for (const auto & roi : target.rois) {
      if (roi.type == "person") {
        bx = roi.rect.x_offset; by = roi.rect.y_offset;
        bw = roi.rect.width; bh = roi.rect.height;
        break;
      }
    }
    float bbox_left = static_cast<float>(bx);
    float bbox_right = (bw > 0) ? bx + static_cast<float>(bw) : 0.0f;
    double margin = edge_margin_ratio_ * image_width_;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Person id: %ld, dist: %.2f m, bbox xywh=(%d,%d,%d,%d), edges=[%.0f,%.0f] (img_w=%.0f, margin=%.0f, edge_band=[0,%.0f] & [%.0f,%.0f])",
      tracking_track_id_, person_map_dist,
      bx, by, bw, bh, bbox_left, bbox_right,
      image_width_, margin, margin, image_width_ - margin, image_width_);
  }

  // Stop following: person closer than follow_distance_min — cancel any active
  // follow nav so the robot stops closing in, and publish a zero cmd_vel as a
  // hard stop fallback (covers inertia / residual velocity from a just-completed
  // goal). Does NOT return: falling through so the edge-turn branch below can
  // still spin the robot toward a too-close target that has slipped to the
  // frame edge (edge-turn now drives /cmd_vel directly, not Nav2, so the cancel
  // here no longer races with the edge-turn goal).
  if (person_map_dist < follow_distance_min_) {
    if (nav_goal_handle_) {
      RCLCPP_INFO(this->get_logger(),
        "Person %.2f m < follow_distance_min %.2f m, canceling nav to stop follow",
        person_map_dist, follow_distance_min_);
      nav_client_->async_cancel_goal(nav_goal_handle_);
      nav_goal_handle_ = nullptr;
      last_nav_goal_pose_ = nullptr;
    }
    if (cmd_vel_pub_) {
      // Mutex with edge-turn: skip zero-vel while a spin is running OR this
      // frame is about to start one, so the spin's angular velocity owns
      // /cmd_vel (otherwise the zero-vel and the spin's angular-z race for
      // <100ms). will_edge_turn mirrors the edge-turn branch condition below.
      const bool will_edge_turn = !spin_active_.load() && !goal_pending_ &&
                                  isTargetAtFrameEdge(target);
      if (!spin_active_.load() && !will_edge_turn) {
        geometry_msgs::msg::Twist stop_twist;
        cmd_vel_pub_->publish(stop_twist);
      }
    }
  }

  // Edge-of-frame turn: target locked (TRACKING), person's ROI at left/right
  // image edge. Turn the robot toward the person to bring them back to frame
  // center. Done via direct cmd_vel spin (SpinInPlaceWCmdVel on a detached
  // thread), NOT Nav2 — a turn-in-place is a short angular impulse, not a nav
  // goal, and going through Nav2 raced with the dist<min cancel above (cancel
  // async + new goal → Nav2 rejected the turn → loop every frame).
  // Mutex: cancel any active Nav2 goal first so the spin owns /cmd_vel
  // exclusively (Nav2 controller also publishes /cmd_vel). Guarded by
  // !spin_active_ (no stacked spins) and !goal_pending_ (no in-flight goal).
  // Closed-loop turn angle: rotate exactly the yaw error to face the target —
  // one spin lands the robot facing the person, so the target returns to frame
  // center without re-spin churn.
  if (!spin_active_.load() && !goal_pending_ && isTargetAtFrameEdge(target) &&
    person_map_dist < follow_distance_min_) {
    // Release /cmd_vel from Nav2 before spinning.
    if (nav_goal_handle_) {
      nav_client_->async_cancel_goal(nav_goal_handle_);
      nav_goal_handle_ = nullptr;
      last_nav_goal_pose_ = nullptr;
    }
    double robot_yaw = tf2::getYaw(current_pose.pose.orientation);
    double yaw_to_target = std::atan2(
      transformed_target_pose.pose.position.y - current_pose.pose.position.y,
      transformed_target_pose.pose.position.x - current_pose.pose.position.x);
    float turn_angle = static_cast<float>(
      angles::shortest_angular_distance(robot_yaw, yaw_to_target));
    RCLCPP_INFO(this->get_logger(),
      "Target at frame edge, turning %.2f rad via cmd_vel toward target at (%.2f, %.2f)",
      turn_angle, transformed_target_pose.pose.position.x,
      transformed_target_pose.pose.position.y);
    spin_active_ = true;
    spin_stop_requested_ = false;
    spin_thread_ = std::make_shared<std::thread>([this, turn_angle]() {
      SpinInPlaceWCmdVel(turn_angle);
      spin_active_ = false;
    });
    spin_thread_->detach();
    setFollowStatus(FollowStatus::TRACKING_EDGE_TURN);
    publishPredictTrajectory();
    return;
  }

  // Withhold vs follow with hysteresis on the band edge: enter follow at
  // dist > max+hyst, exit follow at dist < max-hyst. Without this, dist jitters
  // near follow_distance_max_ and sends/cancels nav every frame (churn).
  // following_active_ holds the sticky state (true=in-follow, false=in-withhold).
  // Reset to false at every "start following a (new) target" point (see the
  // resets alongside track_state_/tracking_track_id_ assignments).
  {
    const double max_enter = follow_distance_max_ + follow_hysteresis_;
    const double max_exit = follow_distance_max_ - follow_hysteresis_;
    if (following_active_) {
      if (person_map_dist < max_exit) following_active_ = false;  // exit follow → withhold
    } else {
      if (person_map_dist > max_enter) following_active_ = true;   // enter follow
    }
  }
  if (!following_active_) {
    // Withhold goal: within the follow band (with hysteresis) — don't send a
    // new goal; let any existing nav continue. Robot neither closes in nor chases.
    setFollowStatus(FollowStatus::TRACKING);
    publishPredictTrajectory();
    return;
  }

  // following_active_==true → person_map_dist effectively > follow_distance_max
  // (with hysteresis): follow — nav goal = the person's position if free; else
  // the nearest-free cell toward the robot.
  // Mutex with edge-turn spin: if a spin is still running, do NOT send a follow
  // nav goal this tick (Nav2 controller also writes /cmd_vel; sending a nav
  // while the spin thread is publishing angular velocity races them and
  // produces a spurious goal + state oscillation TRACKING_EDGE_TURN ↔ TRACKING).
  // Just request stop and return; the next frame, after spin_active_ clears,
  // either edge-turn (if still at frame edge) or follow nav will run.
  if (spin_active_.load()) {
    spin_stop_requested_ = true;
    return;
  }
  geometry_msgs::msg::PoseStamped goal_pose =
    getFreePersonGoal(current_pose, transformed_target_pose);
  bool goal_reachable = !goal_pose.header.frame_id.empty();
  if (!goal_reachable) {
    RCLCPP_INFO(this->get_logger(), "Person goal not reachable, skipping nav this tick");
  }
  setFollowStatus(FollowStatus::TRACKING);

  // Step 1: Check if goal differs enough from current robot pose to warrant navigation.
  // Skip sending a nav goal if the robot is already close enough (distance and yaw).
  bool should_nav = goal_reachable;

  if (!goal_reachable) {
    // No reachable trailing goal this tick — skip all nav-decision logic.
    should_nav = false;
  } else {
    float dist_diff = std::hypot(goal_pose.pose.position.x - current_pose.pose.position.x,
      goal_pose.pose.position.y - current_pose.pose.position.y);
    float yaw_diff = std::abs(angles::shortest_angular_distance(
      tf2::getYaw(current_pose.pose.orientation),
      tf2::getYaw(goal_pose.pose.orientation)));
    if (dist_diff < follow_goal_dist_deadzone_ && yaw_diff < follow_goal_yaw_deadzone_) {
      should_nav = false;
      RCLCPP_INFO(this->get_logger(),
        "should_nav=false: deadzone (dist_diff=%.2f<%.2f && yaw_diff=%.2f<%.2f)",
        dist_diff, follow_goal_dist_deadzone_, yaw_diff, follow_goal_yaw_deadzone_);
    }

    // G13: when no nav goal is active (the last one already succeeded/finished),
    // suppress re-sending a new goal if it barely differs from the last sent goal
    // (distance AND yaw both below thresholds). This avoids frequent Nav2 requests
    // when the person moves slowly — the re-send branch above already handles the
    // "moved enough" case while a goal is active.
    if (should_nav && !nav_goal_handle_ && !goal_pending_ && last_nav_goal_pose_) {
      float dx = goal_pose.pose.position.x - last_nav_goal_pose_->pose.position.x;
      float dy = goal_pose.pose.position.y - last_nav_goal_pose_->pose.position.y;
      float last_dist_diff = std::sqrt(dx * dx + dy * dy);
      float last_yaw_diff = std::abs(angles::shortest_angular_distance(
        tf2::getYaw(last_nav_goal_pose_->pose.orientation),
        tf2::getYaw(goal_pose.pose.orientation)));
      if (last_dist_diff < follow_goal_dist_deadzone_ && last_yaw_diff < follow_goal_yaw_deadzone_) {
        should_nav = false;
        RCLCPP_DEBUG(this->get_logger(),
          "should_nav=false: G13 too close to last goal (last_dist_diff=%.2f<%.2f && last_yaw_diff=%.2f<%.2f)",
          last_dist_diff, follow_goal_dist_deadzone_, last_yaw_diff, follow_goal_yaw_deadzone_);
      }
    }

    // G10: time-based rate limiting. Even if the spatial deadzone says "send",
    // don't re-send goals faster than follow_goal_pub_rate_ Hz, to avoid planner churn.
    if (should_nav) {
      double min_interval = (follow_goal_pub_rate_ > 0.0) ? (1.0 / follow_goal_pub_rate_) : 0.0;
      double since_last_send = (this->now() - last_goal_send_time_).seconds();
      if (goal_send_armed_ && since_last_send < min_interval) {
        should_nav = false;
        RCLCPP_INFO(this->get_logger(),
          "should_nav=false: rate limit (since_last_send=%.2fs < min_interval=%.2fs)",
          since_last_send, min_interval);
      }
    }

    // (B) Unconditional re-nav when the active goal target has moved: if a nav
    // goal is already active and the computed goal moved beyond the position/yaw
    // thresholds vs the last sent goal, cancel + resend immediately, regardless
    // of the should_nav deadzone/rate-limit gate below (which only gates the
    // initial new-goal send when no nav is active).
    if (nav_goal_handle_ && last_nav_goal_pose_) {
      float dx = goal_pose.pose.position.x - last_nav_goal_pose_->pose.position.x;
      float dy = goal_pose.pose.position.y - last_nav_goal_pose_->pose.position.y;
      float nav_dist_diff = std::sqrt(dx * dx + dy * dy);
      float nav_yaw_diff = std::abs(angles::shortest_angular_distance(
        tf2::getYaw(last_nav_goal_pose_->pose.orientation),
        tf2::getYaw(goal_pose.pose.orientation)));
      if (nav_dist_diff > follow_goal_dist_deadzone_ || nav_yaw_diff > follow_goal_yaw_deadzone_) {
        // Rate-limit the goal-changed re-send too (not just the initial send below):
        // without this, every frame the active goal moved beyond thresholds cancels
        // + resends, which under fast target motion floods Nav2 (tick-rate exceeded)
        // and causes ComputePathToPose timeouts. Wait at least follow_goal_pub_rate_'s interval.
        double min_interval = (follow_goal_pub_rate_ > 0.0) ? (1.0 / follow_goal_pub_rate_) : 0.0;
        double since_last_send = (this->now() - last_goal_send_time_).seconds();
        if (goal_send_armed_ && since_last_send < min_interval) {
          RCLCPP_DEBUG(this->get_logger(),
            "Goal changed (dist: %.2f, yaw: %.2f deg) but rate-limited (%.2fs < %.2fs), skip re-send",
            nav_dist_diff, angles::to_degrees(nav_yaw_diff), since_last_send, min_interval);
        } else {
          RCLCPP_INFO(this->get_logger(),
            "Goal changed (dist: %.2f, yaw: %.2f deg), sending new goal (no cancel for smooth transition)",
            nav_dist_diff, angles::to_degrees(nav_yaw_diff));
          // Don't cancel the old goal — Nav2's NavigateToPose action server
          // auto-aborts the active goal when a new one arrives, so the robot
          // transitions smoothly to the new goal without the stop-start pause
          // that an explicit cancel+resend causes. The stale ABORTED result
          // from the old goal is filtered in resultCallback (goal_pending_/
          // handle-mismatch guard).
          NavigateToPose::Goal nav_goal;
          nav_goal.pose = goal_pose;
          asyncNavToGoal(nav_goal);
          last_nav_goal_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(goal_pose);
          last_goal_send_time_ = this->now();
          goal_send_armed_ = true;
        }
      }
    } else if (should_nav) {
      // No active navigation — send a new nav goal (still gated by deadzone + rate-limit)
      RCLCPP_DEBUG(this->get_logger(),
        "Nav to goal at %s frame xy: (%.2f, %.2f).",
        goal_pose.header.frame_id.data(),
        goal_pose.pose.position.x, goal_pose.pose.position.y);
      NavigateToPose::Goal nav_goal;
      nav_goal.pose = goal_pose;
      asyncNavToGoal(nav_goal);
      last_nav_goal_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(goal_pose);
      last_goal_send_time_ = this->now();
      goal_send_armed_ = true;
    }
  }

  if (goal_pose_pub_ && goal_reachable) {
    float dist_diff_log = std::hypot(goal_pose.pose.position.x - current_pose.pose.position.x,
      goal_pose.pose.position.y - current_pose.pose.position.y);
    float yaw_diff_log = std::abs(angles::shortest_angular_distance(
      tf2::getYaw(current_pose.pose.orientation),
      tf2::getYaw(goal_pose.pose.orientation)));
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Tracking person id: %ld, confidence: %.2f, at cam frame xy: (%.2f, %.2f), w: %.2f, h: %.2f."
      " Pub goal pose at %s frame xy: (%.2f, %.2f), current_pose xy: (%.2f, %.2f)."
      " should_nav: %s, dist_diff: %.2f (thr: %.2f), yaw_diff: %.2f (%.0f deg, thr: %.2f),"
      " person_map_dist: %.2f, follow_band=[%.2f, %.2f], nav_active: %s, last_nav_goal: %s",
      target.track_id, confidence,
      pose_camera.pose.position.x, pose_camera.pose.position.y, width, height,
      goal_pose.header.frame_id.data(),
      goal_pose.pose.position.x, goal_pose.pose.position.y,
      current_pose.pose.position.x, current_pose.pose.position.y,
      (should_nav ? "true" : "false"),
      dist_diff_log, follow_goal_dist_deadzone_, yaw_diff_log, angles::to_degrees(yaw_diff_log), follow_goal_yaw_deadzone_,
      person_map_dist, follow_distance_min_, follow_distance_max_,
      (nav_goal_handle_ ? "yes" : "no"),
      (last_nav_goal_pose_ ? "yes" : "no"));
    goal_pose_pub_->publish(goal_pose);
  }

  // Publish the predicted trajectory every frame while tracking, so the
  // prediction quality can be visualized in RViz continuously.
  publishPredictTrajectory();
}

geometry_msgs::msg::PoseStamped PersonFollowingNode::getCurrentPose()
{
  // Look up the latest TF from robot_frame_ to global_frame_ to get the robot's pose.
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = global_frame_;
  robot_pose.header.stamp = this->now();

  try {
    auto transform = tf_buffer_.lookupTransform(
      global_frame_, robot_frame_, tf2::TimePointZero);
    robot_pose.pose.position.x = transform.transform.translation.x;
    robot_pose.pose.position.y = transform.transform.translation.y;
    robot_pose.pose.position.z = transform.transform.translation.z;
    robot_pose.pose.orientation = transform.transform.rotation;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Failed to get robot pose: %s", ex.what());
  }

  return robot_pose;
}

geometry_msgs::msg::PoseStamped PersonFollowingNode::getForwardPose(
  const geometry_msgs::msg::PoseStamped & base,
  float forward_dist)
{
  // Project a pose forward along its orientation by the given distance.
  auto theta = tf2::getYaw(base.pose.orientation);
  auto forward_pose = base;
  forward_pose.pose.position.x += forward_dist * std::cos(theta);
  forward_pose.pose.position.y += forward_dist * std::sin(theta);
  return forward_pose;
}

geometry_msgs::msg::PoseStamped PersonFollowingNode::getTrailingPose(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::PoseStamped & person_pose,
  double follow_distance)
{
  // Compute a trailing goal: the point `follow_distance` behind the person along
  // the robot→person line, oriented to face the person. Both poses are in the
  // global (map) frame. This replaces the old robot-centric forward projection
  // so the robot settles a fixed distance behind the person rather than a
  // distance proportional to how far the person currently is.
  double dx = person_pose.pose.position.x - robot_pose.pose.position.x;
  double dy = person_pose.pose.position.y - robot_pose.pose.position.y;
  double dist = std::hypot(dx, dy);
  // Unit vector from robot toward person; guard against divide-by-zero.
  double ux = (dist > 1e-6) ? dx / dist : 1.0;
  double uy = (dist > 1e-6) ? dy / dist : 0.0;

  geometry_msgs::msg::PoseStamped trailing = person_pose;
  trailing.pose.position.x = person_pose.pose.position.x - ux * follow_distance;
  trailing.pose.position.y = person_pose.pose.position.y - uy * follow_distance;
  // Face toward the person from the trailing point.
  // The trailing point sits on the robot→person ray, behind the person by
  // follow_distance, so the trailing→person direction equals (ux, uy).
  double yaw_to_person = std::atan2(uy, ux);
  trailing.pose.orientation =
    nav2_util::geometry_utils::orientationAroundZAxis(yaw_to_person);
  return trailing;
}

bool PersonFollowingNode::isGoalReachable(const geometry_msgs::msg::PoseStamped & goal)
{
  // Reachable = goal cell is free in the latest costmap. Cheaper than Nav2
  // ComputePathToPose (us vs ~1s with possible timeout/abort churn — see
  // git history for the prior timeout → assume-reachable trade-off). Full A*
  // path validation is left to Nav2 itself when it receives the actual
  // NavigateToPose: if a chosen point is free-cell but mid-path is blocked,
  // Nav2 will abort that goal at execution time (acceptable: same outcome as
  // before, but without the upfront 1s wait and misleading error logs).
  if (!isPointFree(goal.pose.position)) {
    RCLCPP_INFO(this->get_logger(),
      "isGoalReachable: goal (%.2f, %.2f) cell not free in costmap, treating as unreachable",
      goal.pose.position.x, goal.pose.position.y);
    return false;
  }
  return true;
}

void PersonFollowingNode::globalCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  latest_costmap_ = msg;
}

bool PersonFollowingNode::isPointFree(const geometry_msgs::msg::Point & pt)
{
  nav_msgs::msg::OccupancyGrid::SharedPtr cm;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    cm = latest_costmap_;
  }
  int mx, my;
  if (!worldToCell(cm, pt, mx, my)) {
    return false;
  }
  int c = cellCost(cm, mx, my);
  // -1 (OOB/unknown sentinel) and >= thr are not free. Free = [0, thr).
  return c >= 0 && static_cast<double>(c) < costmap_free_cost_thr_;
}

bool PersonFollowingNode::worldToCell(
  const nav_msgs::msg::OccupancyGrid::SharedPtr & cm,
  const geometry_msgs::msg::Point & pt, int & mx, int & my)
{
  if (!cm || cm->info.resolution <= 0.0) {
    return false;
  }
  double ox = cm->info.origin.position.x;
  double oy = cm->info.origin.position.y;
  double res = cm->info.resolution;
  mx = static_cast<int>((pt.x - ox) / res);
  my = static_cast<int>((pt.y - oy) / res);
  if (mx < 0 || my < 0 ||
      mx >= static_cast<int>(cm->info.width) ||
      my >= static_cast<int>(cm->info.height)) {
    return false;  // out of map bounds
  }
  return true;
}

int PersonFollowingNode::cellCost(
  const nav_msgs::msg::OccupancyGrid::SharedPtr & cm, int mx, int my)
{
  if (!cm) {
    return -1;
  }
  if (mx < 0 || my < 0 ||
      mx >= static_cast<int>(cm->info.width) ||
      my >= static_cast<int>(cm->info.height)) {
    return -1;  // out of map bounds
  }
  // Recover the unsigned costmap_2d cost: int8_t 254 (LETHAL) reads as -2 when
  // signed, so cast through uint8_t first to get 254 back as an int.
  size_t idx = static_cast<size_t>(my) * cm->info.width + static_cast<size_t>(mx);
  return static_cast<int>(static_cast<uint8_t>(cm->data[idx]));
}

bool PersonFollowingNode::nearestFreeCellToPerson(
  const geometry_msgs::msg::Point & person_xy,
  double max_radius_m, geometry_msgs::msg::Point & out)
{
  nav_msgs::msg::OccupancyGrid::SharedPtr cm;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    cm = latest_costmap_;
  }
  if (!cm || cm->info.resolution <= 0.0) {
    return false;
  }
  int px, py;
  if (!worldToCell(cm, person_xy, px, py)) {
    return false;  // person outside costmap: cannot center the search
  }
  double res = cm->info.resolution;
  int R = (max_radius_m > 0.0)
    ? static_cast<int>(std::ceil(max_radius_m / res)) : 0;
  // No hard cap — callers may pass a large radius (e.g. max(robot↔person, 2.0m))
  // when the person is far, and the BFS needs to cover that full range.
  int W = 2 * R + 1;
  std::vector<char> visited(static_cast<size_t>(W) * static_cast<size_t>(W), 0);
  std::queue<std::pair<int, int>> q;
  // Seed BFS at the person's cell.
  q.push({px, py});
  visited[static_cast<size_t>(R) * static_cast<size_t>(W) + static_cast<size_t>(R)] = 1;
  const int neighbour_dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int neighbour_dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  while (!q.empty()) {
    auto [cx, cy] = q.front();
    q.pop();
    int c = cellCost(cm, cx, cy);
    if (c >= 0 && static_cast<double>(c) < costmap_free_cost_thr_) {
      // Found the nearest free cell (Chebyshev). Convert to world cell centre.
      out.x = cm->info.origin.position.x + (cx + 0.5) * res;
      out.y = cm->info.origin.position.y + (cy + 0.5) * res;
      out.z = 0.0;
      return true;
    }
    for (int i = 0; i < 8; ++i) {
      int nx = cx + neighbour_dx[i];
      int ny = cy + neighbour_dy[i];
      if (nx < px - R || nx > px + R || ny < py - R || ny > py + R) {
        continue;  // outside the search window
      }
      size_t vidx = static_cast<size_t>(ny - (py - R)) * static_cast<size_t>(W) +
                    static_cast<size_t>(nx - (px - R));
      if (visited[vidx]) {
        continue;
      }
      visited[vidx] = 1;
      q.push({nx, ny});
    }
  }
  return false;
}

geometry_msgs::msg::PoseStamped PersonFollowingNode::getFreePersonGoal(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::PoseStamped & person_pose)
{
  // Goal = the person's position if it's free; else BFS outward from the person
  // for the nearest free cell so Nav2 can route there to a point near the person.
  // Falls back to the person position if no costmap / no free cell is found.
  if (isPointFree(person_pose.pose.position)) {
    return getTrailingPose(robot_pose, person_pose, 0.0);
  }
  // Person cell not free (person standing on an obstacle/unknown cell, or a wall
  // blocks the robot→person line). Find the nearest free cell around the person
  // (BFS, bounded by costmap_free_search_radius_m_) so the goal lands near the
  // person, NOT at the robot's feet. The old ray-retreat toward the robot could
  // fall back to a free cell right next to the robot when the person cell was
  // blocked, producing a ~0m dist_diff that the deadzone gate then suppressed —
  // stranding the robot while the person was still 2+ m away. BFS toward the
  // person fixes that: the goal is near the person (≈ person_map_dist), so the
  // deadzone does not fire and the robot actually closes in.
  // Search radius = max(robot↔person distance, costmap_free_search_radius_m_) so
  // when the person is far the BFS covers enough area to find a free cell.
  double dist_rp = std::hypot(
    person_pose.pose.position.x - robot_pose.pose.position.x,
    person_pose.pose.position.y - robot_pose.pose.position.y);
  double search_radius = std::max(dist_rp, costmap_free_search_radius_m_);
  geometry_msgs::msg::Point free_pt;
  if (nearestFreeCellToPerson(person_pose.pose.position, search_radius, free_pt))
  {
    // Build the goal directly — do NOT use getTrailingPose: it offsets along the
    // robot→person ray, but the BFS point is generally not on that ray.
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = global_frame_;
    goal.header.stamp = person_pose.header.stamp;
    goal.pose.position = free_pt;
    goal.pose.position.z = 0.0;
    // Orient to face the person (same semantics as getTrailingPose's
    // trailing→person direction).
    double yaw = std::atan2(person_pose.pose.position.y - free_pt.y,
                            person_pose.pose.position.x - free_pt.x);
    goal.pose.orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(yaw);
    RCLCPP_INFO(this->get_logger(),
      "person cell not free; using nearest free cell near person at (%.2f, %.2f)",
      goal.pose.position.x, goal.pose.position.y);
    return goal;
  }
  // No free cell within the search radius — the person is in an obstacle-dense
  // region with no reachable nearby free point. Previously this fell back to the
  // person position itself, but that goal lands on a non-free cell and Nav2
  // aborts it every frame (planner fails + churn). Return empty (frame_id empty
  // = not reachable) so publishGoalPose skips sending this tick — no Nav2 churn,
  // and the robot resumes following once the person moves to a free region.
  RCLCPP_INFO(this->get_logger(),
    "no free cell within %.2f m of person (search radius = max(robot↔person, %.2f)); skipping nav this tick (avoid Nav2 abort churn)",
    search_radius, costmap_free_search_radius_m_);
  geometry_msgs::msg::PoseStamped empty;  // frame_id stays empty = not reachable
  return empty;
}

bool PersonFollowingNode::isTargetAtFrameEdge(const ai_msgs::msg::Target & target)
{
  // True if the person detection-box left or right pixel edge falls in the
  // left/right edge band of the image (fraction `edge_margin_ratio_` of
  // `image_width_` on each side). Using the box edges (not its center) means a
  // target is considered at the edge as soon as its near edge reaches the band,
  // which triggers the edge-turn earlier to keep the person in view.
  for (const auto & roi : target.rois) {
    if (roi.type != "person") {
      continue;
    }
    const auto & r = roi.rect;
    if (r.width == 0u || image_width_ <= 0.0) {
      continue;  // invalid ROI / image width not configured
    }
    double left_x = static_cast<double>(r.x_offset);
    double right_x = static_cast<double>(r.x_offset) + r.width;
    double margin = edge_margin_ratio_ * image_width_;
    if (left_x < margin || right_x > (image_width_ - margin)) {
      return true;  // box edge in left or right edge band
    }
    return false;
  }
  return false;  // no person ROI
}

geometry_msgs::msg::PoseStamped PersonFollowingNode::transformPersonToWorld(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::PoseStamped & pose_camera)
{
  // Mirror of the camera→map transform in publishGoalPose. On TF failure
  // returns a PoseStamped whose header.frame_id is empty (caller's sentinel).
  auto transformed = pose_camera;
  if (robot_pose.header.frame_id != transformed.header.frame_id) {
    if (!nav2_util::transformPoseInTargetFrame(
        pose_camera, transformed, tf_buffer_, robot_pose.header.frame_id))
    {
      RCLCPP_DEBUG(this->get_logger(),
        "transformPersonToWorld: could not transform %s to %s",
        transformed.header.frame_id.c_str(), robot_pose.header.frame_id.c_str());
      geometry_msgs::msg::PoseStamped empty;
      return empty;
    }
    transformed.header.frame_id = robot_pose.header.frame_id;
  }
  return transformed;
}

double PersonFollowingNode::targetMovementOverWindow(uint64_t id, double window_sec)
{
  auto it = target_pos_history_.find(id);
  if (it == target_pos_history_.end() || it->second.size() < 2) {
    return -1.0;
  }
  const auto & samples = it->second;
  rclcpp::Time now = this->now();
  // Newest sample in the window.
  const TargetPositionSample * newest = nullptr;
  const TargetPositionSample * oldest_in_window = nullptr;
  for (const auto & s : samples) {
    if ((now - s.stamp).seconds() <= window_sec) {
      if (!oldest_in_window) {
        oldest_in_window = &s;
      }
      newest = &s;
    }
  }
  if (!newest || !oldest_in_window || newest == oldest_in_window) {
    return -1.0;
  }
  double dx = newest->world_pos.x - oldest_in_window->world_pos.x;
  double dy = newest->world_pos.y - oldest_in_window->world_pos.y;
  return std::hypot(dx, dy);
}

void PersonFollowingNode::pruneTargetPosHistory(double max_age_sec)
{
  rclcpp::Time now = this->now();
  for (auto & kv : target_pos_history_) {
    auto & samples = kv.second;
    while (!samples.empty() && (now - samples.front().stamp).seconds() > max_age_sec) {
      samples.erase(samples.begin());
    }
  }
  // Drop ids with no samples left.
  for (auto it = target_pos_history_.begin(); it != target_pos_history_.end(); ) {
    if (it->second.empty()) {
      it = target_pos_history_.erase(it);
    } else {
      ++it;
    }
  }
}

geometry_msgs::msg::PoseStamped PersonFollowingNode::predictTargetPose(uint64_t id)
{
  // Predict the person's world-frame position by extrapolating recent velocity.
  // Returns empty PoseStamped (header.frame_id empty) when no usable prediction:
  // too few samples, stationary (no meaningful direction), or stale newest sample.
  geometry_msgs::msg::PoseStamped empty;  // frame_id stays empty as the sentinel
  auto it = target_pos_history_.find(id);
  if (it == target_pos_history_.end() || it->second.size() < 2) {
    return empty;
  }
  const auto & samples = it->second;
  const auto & newest = samples.back();
  rclcpp::Time now = this->now();
  // Skip if the newest sample is too stale (person not seen for a while).
  if ((now - newest.stamp).seconds() > predict_stale_sec_) {
    return empty;
  }
  // Find the oldest sample within the velocity-estimation window.
  const TargetPositionSample * oldest_in_window = nullptr;
  for (const auto & s : samples) {
    if ((now - s.stamp).seconds() <= predict_window_sec_) {
      oldest_in_window = &s;
      break;
    }
  }
  if (!oldest_in_window || oldest_in_window == &newest) {
    return empty;
  }
  double dt = (newest.stamp - oldest_in_window->stamp).seconds();
  if (dt <= 1e-6) {
    return empty;
  }
  double vx = (newest.world_pos.x - oldest_in_window->world_pos.x) / dt;
  double vy = (newest.world_pos.y - oldest_in_window->world_pos.y) / dt;
  double speed = std::hypot(vx, vy);
  // Stationary -> no meaningful direction; skip (reuse the stationary threshold).
  if (speed < static_target_move_thr_) {
    return empty;
  }
  // Extrapolate: newest_pos + v * (time since newest + lead), clamped to max dist.
  double since = (now - newest.stamp).seconds();
  double disp_x = vx * (since + predict_lead_sec_);
  double disp_y = vy * (since + predict_lead_sec_);
  double disp = std::hypot(disp_x, disp_y);
  if (disp > predict_max_dist_) {
    double scale = predict_max_dist_ / disp;
    disp_x *= scale;
    disp_y *= scale;
  }
  geometry_msgs::msg::PoseStamped predicted;
  predicted.header.frame_id = global_frame_;
  predicted.header.stamp = now;
  predicted.pose.position.x = newest.world_pos.x + disp_x;
  predicted.pose.position.y = newest.world_pos.y + disp_y;
  predicted.pose.position.z = 0.0;
  // Boundary check: reject predictions that fall outside the known costmap.
  // Otherwise an out-of-map goal makes the Nav2 planner spam worldToMap-failed
  // errors and stalls isGoalReachable() for its full timeout. A missing costmap
  // is treated as "cannot bound-check" → keep the prediction (no regression for
  // the no-costmap startup window).
  {
    nav_msgs::msg::OccupancyGrid::SharedPtr cm;
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      cm = latest_costmap_;
    }
    if (cm) {
      int mx, my;
      if (!worldToCell(cm, predicted.pose.position, mx, my)) {
        RCLCPP_INFO(this->get_logger(),
          "LOST; predicted target id=%lu at (%.2f, %.2f) is outside the costmap, not predicting",
          id, predicted.pose.position.x, predicted.pose.position.y);
        return empty;
      }
    }
  }
  // Orient along the person's motion direction.
  double yaw = std::atan2(vy, vx);
  predicted.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
  return predicted;
}

// --- Belief-Guided search (Improvement M) ---

bool PersonFollowingNode::getLastKnownPose(uint64_t id,
  geometry_msgs::msg::Point & lkp, double & vx, double & vy, double & speed)
{
  // Extract raw LKP + velocity from target_pos_history_, without stale filtering
  // (unlike predictTargetPose). M needs the LKP regardless of how old it is.
  auto it = target_pos_history_.find(id);
  if (it == target_pos_history_.end() || it->second.size() < 2) {
    return false;
  }
  const auto & samples = it->second;
  const auto & newest = samples.back();
  lkp = newest.world_pos;

  // Velocity from oldest-in-window → newest (same window as predictTargetPose).
  rclcpp::Time now = this->now();
  const TargetPositionSample * oldest_in_window = nullptr;
  for (const auto & s : samples) {
    if ((now - s.stamp).seconds() <= predict_window_sec_) {
      oldest_in_window = &s;
      break;
    }
  }
  if (!oldest_in_window || oldest_in_window == &newest) {
    vx = 0.0; vy = 0.0; speed = 0.0;
    return true;
  }
  double dt = (newest.stamp - oldest_in_window->stamp).seconds();
  if (dt <= 1e-6) {
    vx = 0.0; vy = 0.0; speed = 0.0;
    return true;
  }
  vx = (newest.world_pos.x - oldest_in_window->world_pos.x) / dt;
  vy = (newest.world_pos.y - oldest_in_window->world_pos.y) / dt;
  speed = std::hypot(vx, vy);
  return true;
}

bool PersonFollowingNode::getRelockLkp(geometry_msgs::msg::Point & lkp) const
{
  // Prefer the belief search's LKP (which may have been rewound from an
  // out-of-costmap history to an in-bounds sample). Fall back to the freshest
  // history sample when belief search is inactive (e.g. after the pending-lost
  // window without an active search). If nothing exists, return false.
  if (belief_search_.active) {
    lkp = belief_search_.lkp;
    return true;
  }
  auto hist_it = target_pos_history_.find(tracking_track_id_);
  if (hist_it == target_pos_history_.end() || hist_it->second.empty()) {
    return false;
  }
  lkp = hist_it->second.back().world_pos;  // freshest sample = LKP
  return true;
}

double PersonFollowingNode::currentRelockDist() const
{
  // LOST 持续时间分段：早期严格（只接受极近重锁，几乎只能是原目标），
  // 晚期放宽（原目标可能已移动较远）。tp_lost_ 在 FLOW 2 enter_lost 设置
  // （cpp:505），LOST 期每帧有效；WILL_BE_LOST 宽限期 tp_lost_ 未设但不走
  // FLOW 7/8（仍在 TRACKING 分支），无影响。同一时间差模式见 cpp:459。
  double t = (this->now() - tp_lost_).seconds();
  if (t < relock_dist_phase1_sec_) return relock_dist_phase1_;
  if (t < relock_dist_phase2_sec_) return relock_dist_phase2_;
  return relock_dist_phase3_;
}

geometry_msgs::msg::PoseStamped PersonFollowingNode::pickLkpObservationPose(bool & from_history)
{
  // Observation point = LKP itself. Only when the robot arrives at the LKP
  // itself does it scan. If the LKP cell is non-free (target in obstacle/unknown
  // region), navigate to the nearest free cell nearby as a transitional step
  // (at_lkp=false → no scan), and retry the LKP itself next round — by then the
  // robot moved closer and the costmap may have updated (LKP cell may now be free).
  from_history = false;
  belief_search_.at_lkp = false;
  geometry_msgs::msg::PoseStamped empty;

  // Level 1: LKP itself is free → nav to LKP, scan on arrival (at_lkp=true).
  if (isPointFree(belief_search_.lkp)) {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = global_frame_;
    goal.header.stamp = this->now();
    goal.pose.position = belief_search_.lkp;
    goal.pose.position.z = 0.0;
    double yaw = std::atan2(belief_search_.vy, belief_search_.vx);
    goal.pose.orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(yaw);
    belief_search_.at_lkp = true;
    RCLCPP_INFO(this->get_logger(),
      "[LOST-FLOW] belief search: LKP (%.2f,%.2f) is free → nav to LKP itself, will scan on arrival",
      belief_search_.lkp.x, belief_search_.lkp.y);
    return goal;
  }

  // Level 2: LKP not free → BFS nearest free cell around LKP (transitional,
  // at_lkp=false → no scan, retry LKP next round). Radius = max(robot↔LKP, 2m).
  {
    auto robot_pose = getCurrentPose();
    double dist_rl = std::hypot(
      belief_search_.lkp.x - robot_pose.pose.position.x,
      belief_search_.lkp.y - robot_pose.pose.position.y);
    double search_radius = std::max(dist_rl, costmap_free_search_radius_m_);
    geometry_msgs::msg::Point free_pt;
    if (nearestFreeCellToPerson(belief_search_.lkp, search_radius, free_pt)) {
      geometry_msgs::msg::PoseStamped goal;
      goal.header.frame_id = global_frame_;
      goal.header.stamp = this->now();
      goal.pose.position = free_pt;
      goal.pose.position.z = 0.0;
      double yaw = std::atan2(belief_search_.lkp.y - free_pt.y,
                             belief_search_.lkp.x - free_pt.x);
      goal.pose.orientation =
        nav2_util::geometry_utils::orientationAroundZAxis(yaw);
      RCLCPP_INFO(this->get_logger(),
        "[LOST-FLOW] belief search: LKP (%.2f,%.2f) not free → transitional free cell (%.2f,%.2f), no scan, will retry LKP next round",
        belief_search_.lkp.x, belief_search_.lkp.y, free_pt.x, free_pt.y);
      return goal;
    }
  }

  // Level 3: LKP not free and no free cell nearby → rewind history.
  // Two-level per sample: sample itself free → use it; sample non-free → BFS around sample.
  auto hist_it = target_pos_history_.find(tracking_track_id_);
  if (hist_it != target_pos_history_.end()) {
    for (auto s_it = hist_it->second.rbegin(); s_it != hist_it->second.rend(); ++s_it) {
      const auto & pt = s_it->world_pos;
      // Two-level search for an observation point near this history sample:
      //  Level 1: the sample point itself is free → use it directly (cheap, no BFS).
      //  Level 2: the sample is non-free (target was in an obstacle/unknown region —
      //           costmap often marks the person's cell as lethal) → BFS around the
      //           sample for the nearest free cell (the target stood at the edge of
      //           an obstacle patch; a free vantage point is usually nearby). This
      //           covers the common case where the whole history is non-free but
      //           free space surrounds the obstacle patch.
      geometry_msgs::msg::Point obs_pt;
      bool found = false;
      if (isPointFree(pt)) {
        obs_pt = pt;
        found = true;
      } else {
        // BFS around the history sample; radius = max(robot↔sample, default 2m)
        // so when the sample is far the BFS covers enough area.
        auto robot_pose = getCurrentPose();
        double dist_rs = std::hypot(pt.x - robot_pose.pose.position.x,
                                    pt.y - robot_pose.pose.position.y);
        double search_radius = std::max(dist_rs, costmap_free_search_radius_m_);
        if (nearestFreeCellToPerson(pt, search_radius, obs_pt)) {
          found = true;
        }
      }
      if (!found) continue;
      geometry_msgs::msg::PoseStamped goal;
      goal.header.frame_id = global_frame_;
      goal.header.stamp = this->now();
      goal.pose.position = obs_pt;
      goal.pose.position.z = 0.0;
      // Orient toward the LKP (look toward where the target was last seen).
      double yaw = std::atan2(belief_search_.lkp.y - obs_pt.y,
                             belief_search_.lkp.x - obs_pt.x);
      goal.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
      RCLCPP_INFO(this->get_logger(),
        "[LOST-FLOW] belief search: no free obs point near LKP; rewinding history to free sample (%.2f, %.2f)",
        obs_pt.x, obs_pt.y);
      from_history = true;
      return goal;
    }
  }
  return empty;
}

void PersonFollowingNode::startBeliefSearch()
{
  // Extract LKP + velocity from history.
  geometry_msgs::msg::Point lkp;
  double vx, vy, speed;
  if (!getLastKnownPose(tracking_track_id_, lkp, vx, vy, speed)) {
    // No motion history (<2 samples): happens when the target was just locked
    // on / just switched to and lost almost immediately. Both belief search
    // (needs LKP + motion) and prediction (predictTargetPose also needs ≥2
    // samples) are unusable, so there is no active search to run — the only
    // recovery is to wait for the LOST timeout (lost_to_idle_timeout_sec_) → IDLE
    // spin. (A prediction here would also fail: predictTargetPose needs ≥2
    // samples too, so staying put directly is equivalent and simpler.)
    RCLCPP_INFO(this->get_logger(),
      "[LOST-FLOW 4] belief search: no motion history (<2 samples) for id=%lu, staying put (wait LOST timeout -> IDLE)",
      tracking_track_id_);
    setFollowStatus(FollowStatus::LOST_HOLDING);  // staying put, no search possible
    return;
  }

  // LKP out-of-bounds recovery: when the target walked off the edge of the
  // known costmap before being lost, the LKP is outside the map and every
  // search centered on it fails (pickLkpObservationPose finds no free cell,
  // predictTargetPose extrapolates further out of bounds). Recover by walking
  // the history backward (newest→oldest) for the most recent sample still
  // inside the costmap, and use that as the search center. Motion direction is
  // recomputed as the vector from that in-bounds history sample toward the
  // original LKP — i.e. the direction the target actually traveled — so the
  // observation point is chosen along the real trajectory, not a velocity
  // extrapolation that points further off the map. Falls back to the original
  // LKP/motion if the whole history is out of bounds (rare).
  {
    nav_msgs::msg::OccupancyGrid::SharedPtr cm;
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      cm = latest_costmap_;
    }
    int lkp_mx, lkp_my;
    if (cm && !worldToCell(cm, lkp, lkp_mx, lkp_my)) {
      auto hist_it = target_pos_history_.find(tracking_track_id_);
      if (hist_it != target_pos_history_.end()) {
        const geometry_msgs::msg::Point * in_bounds = nullptr;
        for (auto s_it = hist_it->second.rbegin(); s_it != hist_it->second.rend(); ++s_it) {
          int mx, my;
          if (worldToCell(cm, s_it->world_pos, mx, my)) {
            in_bounds = &s_it->world_pos;
            break;
          }
        }
        if (in_bounds) {
          double dx = lkp.x - in_bounds->x;
          double dy = lkp.y - in_bounds->y;
          double dlen = std::hypot(dx, dy);
          RCLCPP_INFO(this->get_logger(),
            "[LOST-FLOW 6] LKP (%.2f,%.2f) out of costmap; rewinding history to in-bounds sample (%.2f,%.2f) as search center",
            lkp.x, lkp.y, in_bounds->x, in_bounds->y);
          setFollowStatus(FollowStatus::LOST_REWIND);  // signal LKP was rewound (distinct from plain LOST)
          lkp = *in_bounds;
          // Recompute motion along the actual traveled trajectory (history→LKP),
          // preserving the original speed magnitude so has_motion is consistent.
          if (dlen > 1e-6) {
            vx = (dx / dlen) * speed;
            vy = (dy / dlen) * speed;
          }
          // else: degenerate (history sample == LKP); keep original vx/vy.
        }
      }
    }
  }

  // Initialize belief search state.
  belief_search_.active = true;
  belief_search_.round = 0;
  belief_search_.max_rounds = belief_search_max_rounds_;
  belief_search_.lkp = lkp;
  belief_search_.vx = vx;
  belief_search_.vy = vy;
  belief_search_.speed = speed;
  belief_search_.has_motion = (speed >= static_target_move_thr_);
  belief_search_.searched_points.clear();
  belief_search_.start_time = this->now();
  belief_search_.scanning = false;
  belief_search_.scan_completed = false;

  RCLCPP_INFO(this->get_logger(),
    "[LOST-FLOW 6] belief search started: id=%lu, LKP=(%.2f,%.2f), motion=(%.2f,%.2f), speed=%.2f, has_motion=%s",
    tracking_track_id_, lkp.x, lkp.y, vx, vy, speed,
    (belief_search_.has_motion ? "true" : "false"));

  // Prediction-first (default LOST recovery): try the velocity-extrapolated
  // predicted target position as the first observation point. It points where
  // the target is heading (time-aligned with a moving target), so arriving there
  // gives the camera a direct view of the target if it kept moving — better than
  // scanning near the LKP where the target already left. Falls back to the
  // LKP-centered pickLkpObservationPose (belief search) when prediction is
  // unusable (no samples / stale / stationary / out-of-costmap / unreachable).
  // Both paths feed into the same continueBeliefSearch scan/iterate framework,
  // so a prediction miss naturally continues with LKP-centered search rounds.
  geometry_msgs::msg::PoseStamped obs;
  bool used_prediction = false;
  bool from_history = false;  // set by pickLkpObservationPose when it rewinds history
  belief_search_.at_lkp = false;  // default; pickLkpObservationPath / prediction override below
  auto predicted = predictTargetPose(tracking_track_id_);
  if (!predicted.header.frame_id.empty() && isGoalReachable(predicted)) {
    obs = predicted;
    used_prediction = true;
    belief_search_.at_lkp = true;  // predicted point = target's future position → scan on arrival
    RCLCPP_INFO(this->get_logger(),
      "[LOST-FLOW 6] predicting target id=%lu at (%.2f, %.2f), navigating to predicted point",
      tracking_track_id_, predicted.pose.position.x, predicted.pose.position.y);
  } else {
    // Prediction unusable → belief search fallback: pick an observation point
    // near the LKP (or rewound from history). at_lkp is set inside pickLkpObservationPose.
    obs = pickLkpObservationPose(from_history);
  }

  if (obs.header.frame_id.empty()) {
    // No free observation point near the LKP (and the history rewind also
    // found none). belief search cannot start (nowhere to stand and scan).
    // Stay put and let the LOST timeout (lost_to_idle_timeout_sec_) → IDLE spin
    // handle recovery. (A prediction here would also fail: LKP in an edge/obstacle
    // region means the extrapolated predicted point is almost always out of
    // costmap or unreachable.)
    RCLCPP_INFO(this->get_logger(),
      "[LOST-FLOW 5] belief search: no observation point found (no free cell), staying put (wait LOST timeout -> IDLE)");
    belief_search_.active = false;
    setFollowStatus(FollowStatus::LOST_HOLDING);  // staying put, no free observation point
    return;
  }

  // Cancel any in-flight nav goal, navigate to observation point.
  if (nav_goal_handle_) {
    nav_client_->async_cancel_goal(nav_goal_handle_);
    nav_goal_handle_ = nullptr;
  }
  // Cache robot↔observation-point distance for the LOST_BELIEF_NAV status :dist=.
  {
    auto robot_pose = getCurrentPose();
    double gdx = obs.pose.position.x - robot_pose.pose.position.x;
    double gdy = obs.pose.position.y - robot_pose.pose.position.y;
    last_nav_goal_dist_ = std::hypot(gdx, gdy);
  }
  follow_status_ = FollowStatus::LOST;  // bypass dedupe
  if (used_prediction) {
    setFollowStatus(FollowStatus::LOST_PREDICTING);  // round 0: navigating to predicted point
  } else if (from_history) {
    setFollowStatus(FollowStatus::LOST_BELIEF_NAV_HISTORY);  // round 0: navigating to history-rewound free point
  } else {
    setFollowStatus(FollowStatus::LOST_BELIEF_NAV);  // round 0: navigating to LKP-area observation point
  }
  NavigateToPose::Goal nav_goal;
  nav_goal.pose = obs;
  asyncNavToGoal(nav_goal);
  last_nav_goal_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(obs);
  belief_search_.searched_points.push_back(obs.pose.position);  // record so is_searched excludes it next round
  last_goal_send_time_ = this->now();
  goal_send_armed_ = true;
  // Visualization: when navigating to the predicted point, publish the target's
  // history + predicted point (so RViz shows where the target went + where it's
  // heading); otherwise publish the belief search path (LKP → searched points →
  // current obs). Both reuse predict_trajectory; the branch keeps the two
  // visualizations from clobbering each other.
  if (used_prediction) {
    publishPredictTrajectory();
  } else {
    publishBeliefSearchPath();
  }
  RCLCPP_INFO(this->get_logger(),
    "[LOST-FLOW 6] belief search round 0: navigating to observation point (%.2f, %.2f)",
    obs.pose.position.x, obs.pose.position.y);
}

void PersonFollowingNode::continueBeliefSearch()
{
  if (!belief_search_.active) return;

  // Check timeout.
  if ((this->now() - belief_search_.start_time).seconds() > belief_search_timeout_sec_) {
    RCLCPP_INFO(this->get_logger(),
      "[LOST-FLOW 9] belief search timed out (%.1fs), ending", belief_search_timeout_sec_);
    belief_search_.active = false;
    // Search ended → revert to plain LOST (waiting for LOST timeout → IDLE).
    setFollowStatus(FollowStatus::LOST);
    return;
  }

  // Start a scan if we just arrived at an observation point and haven't scanned
  // here yet (scanning==false, scan_completed==false). scan_completed distinguishes
  // "arrived, scan not started" from "scan finished, advance to next round" — both
  // have scanning==false, so without it the arrival branch would re-fire and we'd
  // never advance past round 0.
  if (!belief_search_.scanning && !belief_search_.scan_completed) {
    // Far-distance skip: if the robot is still far from the LKP (target's last
    // known position), an in-place scan is useless — the camera can't see that
    // far. Skip the scan and advance to the next round to pick a closer
    // observation point and keep moving toward the target.
    {
      auto robot_pose = getCurrentPose();
      double dist_to_lkp = std::hypot(
        belief_search_.lkp.x - robot_pose.pose.position.x,
        belief_search_.lkp.y - robot_pose.pose.position.y);
      if (dist_to_lkp > 1.5) {
        RCLCPP_INFO(this->get_logger(),
          "[LOST-FLOW] belief search: robot %.2f m from LKP — too far to scan, advancing to next round",
          dist_to_lkp);
        belief_search_.scan_completed = true;  // pretend scan done → triggers round++ below
        return;
      }
    }
    // Only scan when we arrived at the LKP itself (at_lkp=true). If we're at a
    // transitional free cell (at_lkp=false), skip the scan and advance to the
    // next round to retry the LKP itself — the robot moved closer and the LKP
    // cell may now be free.
    if (!belief_search_.at_lkp) {
      RCLCPP_INFO(this->get_logger(),
        "[LOST-FLOW] belief search round %d: at transitional cell (not LKP), skip scan, retry LKP next round",
        belief_search_.round);
      belief_search_.scan_completed = true;  // skip scan → triggers round++ below
      return;
    }
    belief_search_.scanning = true;
    spin_stop_requested_ = false;
    spin_active_ = true;
    // Scan direction: toward LKP (bias the spin to the side the target likely is).
    float dir = 1.0f;  // default left
    if (belief_search_.has_motion) {
      // Bias scan toward the side of the motion direction relative to robot.
      auto robot_pose = getCurrentPose();
      double dx = belief_search_.lkp.x - robot_pose.pose.position.x;
      double dy = belief_search_.lkp.y - robot_pose.pose.position.y;
      double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
      double rel_angle = std::atan2(dy, dx) - robot_yaw;
      dir = (std::sin(rel_angle) >= 0) ? 1.0f : -1.0f;
    }
    float final_dir = dir;
    spin_thread_ = std::make_shared<std::thread>([this, final_dir]() {
      SpinInPlaceWCmdVel(spin_radian_ * final_dir);
      spin_active_ = false;
      belief_search_.scanning = false;
      belief_search_.scan_completed = true;  // signal the next call to advance
    });
    spin_thread_->detach();
    RCLCPP_INFO(this->get_logger(),
      "[LOST-FLOW 10] belief search round %d: nav reached, starting directional scan", belief_search_.round);
    // Arrived at the observation point → switch from NAV to SCAN so consumers
    // see the search progressing (otherwise the status would sit on NAV for the
    // whole scan with no visible change).
    setFollowStatus(FollowStatus::LOST_BELIEF_SCAN);
    return;
  }

  // If still scanning, wait. The scan runs on a detached thread and sets
  // scan_completed when done; the LOST per-frame poll (detectResultCallback)
  // re-enters here to advance once it sees scan_completed.
  if (belief_search_.scanning) {
    return;
  }

  // scan_completed: scanning finished at this observation point and the target
  // was not found → advance to the next round.
  belief_search_.scan_completed = false;  // consumed; reset for the next observation point
  belief_search_.round++;
  if (belief_search_.round >= belief_search_.max_rounds) {
    RCLCPP_INFO(this->get_logger(),
      "[LOST-FLOW 12] belief search exhausted %d rounds (max=%d), ending",
      belief_search_.round, belief_search_.max_rounds);
    belief_search_.active = false;
    // Search ended → revert to plain LOST (waiting for LOST timeout → IDLE).
    setFollowStatus(FollowStatus::LOST);
    return;
  }

  // Select next observation point. The just-scanned observation point was
  // already recorded into searched_points when its nav was sent (in
  // startBeliefSearch / below), so pickLkpObservationPose's is_searched filter
  // excludes it — round N picks a different point than round N-1.
  bool from_history = false;
  auto obs = pickLkpObservationPose(from_history);
  if (obs.header.frame_id.empty()) {
    RCLCPP_INFO(this->get_logger(),
      "[LOST-FLOW 12] belief search round %d: no more observation points, ending", belief_search_.round);
    belief_search_.active = false;
    // Search ended → revert to plain LOST (waiting for LOST timeout → IDLE).
    setFollowStatus(FollowStatus::LOST);
    return;
  }

  if (nav_goal_handle_) {
    nav_client_->async_cancel_goal(nav_goal_handle_);
    nav_goal_handle_ = nullptr;
  }
  NavigateToPose::Goal nav_goal;
  nav_goal.pose = obs;
  asyncNavToGoal(nav_goal);
  last_nav_goal_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(obs);
  belief_search_.searched_points.push_back(obs.pose.position);  // record so is_searched excludes it next round
  last_goal_send_time_ = this->now();
  goal_send_armed_ = true;
  publishBeliefSearchPath();
  // Cache robot↔observation-point distance for the LOST_BELIEF_NAV status :dist=.
  {
    auto robot_pose = getCurrentPose();
    double gdx = obs.pose.position.x - robot_pose.pose.position.x;
    double gdy = obs.pose.position.y - robot_pose.pose.position.y;
    last_nav_goal_dist_ = std::hypot(gdx, gdy);
  }
  // Scanning done at this point → advance to next observation point's nav.
  // round already incremented above; switch SCAN → NAV so consumers see progress.
  setFollowStatus(from_history
    ? FollowStatus::LOST_BELIEF_NAV_HISTORY
    : FollowStatus::LOST_BELIEF_NAV);
  RCLCPP_INFO(this->get_logger(),
    "[LOST-FLOW 11] belief search round %d: scanning done, navigating to next observation point (%.2f, %.2f)",
    belief_search_.round, obs.pose.position.x, obs.pose.position.y);
}

void PersonFollowingNode::publishPredictTrajectory()
{
  // Publish the tracked person's trajectory for RViz visualization: recent
  // world-frame history samples (where it actually went) + the extrapolated
  // predicted point (where it's heading, when a usable prediction exists).
  // Called every frame while TRACKING.
  if (!predict_traj_pub_) {
    return;
  }
  nav_msgs::msg::Path traj;
  traj.header.stamp = this->now();
  traj.header.frame_id = global_frame_;
  auto it = target_pos_history_.find(tracking_track_id_);
  if (it != target_pos_history_.end()) {
    for (const auto & s : it->second) {
      geometry_msgs::msg::PoseStamped p;
      p.header = traj.header;
      p.pose.position = s.world_pos;
      traj.poses.push_back(p);
    }
  }
  // Append the predicted point as the last waypoint when available.
  if (tracking_track_id_ != 0) {
    auto predicted = predictTargetPose(tracking_track_id_);
    if (!predicted.header.frame_id.empty()) {
      predicted.header = traj.header;
      traj.poses.push_back(predicted);
    }
  }
  predict_traj_pub_->publish(traj);
}

void PersonFollowingNode::publishBeliefSearchPath()
{
  // Publish the belief-guided search path on "predict_trajectory" for RViz:
  //   LKP (red in viewer) -> already searched observation points -> current
  //   target observation point. Renders the search trajectory as a single
  //   connected Path so the whole search route is visible at a glance, including
  //   previously visited points (verifies the "exclude already searched" step).
  // An empty path (no active search) clears the display.
  if (!predict_traj_pub_) {
    return;
  }
  nav_msgs::msg::Path traj;
  traj.header.stamp = this->now();
  traj.header.frame_id = global_frame_;

  if (belief_search_.active) {
    // Start from LKP, then already-searched points, then the current target
    // observation point (held in last_nav_goal_pose_).
    geometry_msgs::msg::PoseStamped p;
    p.header = traj.header;
    p.pose.position = belief_search_.lkp;
    traj.poses.push_back(p);
    for (const auto & sp : belief_search_.searched_points) {
      p.pose.position = sp;
      traj.poses.push_back(p);
    }
    if (last_nav_goal_pose_) {
      p.pose = last_nav_goal_pose_->pose;
      traj.poses.push_back(p);
    }
  }
  predict_traj_pub_->publish(traj);
}

float PersonFollowingNode::computeTargetScore(const PersonTarget & p)
{
  // Distance factor: closer = higher (normalized by target_filter_range_x_max_).
  float w_dist = 1.0f - std::clamp(static_cast<float>(p.x_m / target_filter_range_x_max_), 0.0f, 1.0f);
  // Score factor: confidence directly (already 0-1).
  float w_score = p.confidence;
  // Center factor: how centered the bbox is in the image.
  float half_w = image_width_ / 2.0f;
  float center_offset = std::abs(p.bbox_center_x - half_w);
  float w_center = 1.0f - std::clamp(center_offset / half_w, 0.0f, 1.0f);
  float score = select_weight_dist_ * w_dist
              + select_weight_score_ * w_score
              + select_weight_center_ * w_center;
  RCLCPP_DEBUG(this->get_logger(),
    "Target id=%lu score=%.3f (dist=%.2f, conf=%.2f, center=%.2f, w=[%.1f,%.1f,%.1f])",
    p.target->track_id, score, w_dist, w_score, w_center,
    select_weight_dist_, select_weight_score_, select_weight_center_);
  return score;
}

void PersonFollowingNode::publishFollowedTarget()
{
  if (!followed_target_pub_) {
    return;
  }
  ai_msgs::msg::PerceptionTargets out;
  out.header = latest_detection_.header;  // keep original ai msg header

  // Encode the follow status as an attribute so consumers can correlate.
  ai_msgs::msg::Attribute status_attr;
  // Status name string (strip "FollowStatus " prefix from followStatusToStr).
  std::string status_str = followStatusToStr(follow_status_);
  const std::string prefix = "FollowStatus ";
  if (status_str.find(prefix) == 0) {
    status_str = status_str.substr(prefix.size());
  }
  status_attr.type = status_str;
  status_attr.value = static_cast<float>(static_cast<int>(follow_status_));
  status_attr.confidence = 1.0f;

  if (tracking_track_id_ != 0) {
    // A target is being followed — publish only that target's full info.
    ai_msgs::msg::Target target;
    target.type = "person";
    target.track_id = tracking_track_id_;
    for (const auto & t : latest_detection_.targets) {
      if (t.track_id == tracking_track_id_) {
        target.rois = t.rois;
        target.attributes = t.attributes;
        break;
      }
    }
    target.attributes.push_back(status_attr);
    out.targets.push_back(target);
  } else {
    // No followed target — forward all detected targets from the latest frame,
    // keeping only track_id, rois (with score/confidence), stripped of
    // attributes/points/captures to keep the message lightweight.
    for (const auto & t : latest_detection_.targets) {
      if (t.type != "person") continue;
      ai_msgs::msg::Target stripped;
      stripped.type = "person";
      stripped.track_id = t.track_id;
      stripped.rois = t.rois;  // roi.confidence carries the detection score
      stripped.attributes.push_back(status_attr);
      out.targets.push_back(stripped);
    }
  }

  followed_target_pub_->publish(out);
}

void PersonFollowingNode::publishFollowedTargetPose(
  const geometry_msgs::msg::PoseStamped & target_pose)
{
  if (!followed_target_pose_pub_ || target_pose.header.frame_id.empty()) {
    return;
  }
  followed_target_pose_pub_->publish(target_pose);
}

geometry_msgs::msg::PoseStamped PersonFollowingNode::getPointingPose(
  const geometry_msgs::msg::PoseStamped & base,
  const geometry_msgs::msg::PoseStamped & goal)
{
  // Compute a pose at base's position but oriented to face toward goal.
  float dx = goal.pose.position.x - base.pose.position.x;
  float dy = goal.pose.position.y - base.pose.position.y;
  float yaw_to_goal = std::atan2(dy, dx);

  auto pointing_pose = base;
  pointing_pose.pose.orientation =
    nav2_util::geometry_utils::orientationAroundZAxis(yaw_to_goal);
  return pointing_pose;
}

void PersonFollowingNode::publishBlindZoneObserving(bool enable)
{
  // Toggle the blind-zone observing planner via a Bool topic.
  // true  → resume observing (used when not tracking or after losing a target)
  // false → pause observing (used while actively tracking a person)
  std_msgs::msg::Bool msg;
  msg.data = enable;
  enable_blind_zone_observing_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Published enable_blind_zone_observing: %s",
    enable ? "true" : "false");
}

std::string PersonFollowingNode::followStatusToStr(FollowStatus s)
{
  switch (s) {
    case FollowStatus::DISABLED:           return "FollowStatus DISABLED";
    case FollowStatus::IDLE_SEARCHING:      return "FollowStatus IDLE_SEARCHING";
    case FollowStatus::IDLE_OBSERVING:      return "FollowStatus IDLE_OBSERVING";
    case FollowStatus::TRACKING:           return "FollowStatus TRACKING";
    case FollowStatus::TRACKING_TOO_CLOSE: return "FollowStatus TRACKING_TOO_CLOSE";
    case FollowStatus::TRACKING_EDGE_TURN: return "FollowStatus TRACKING_EDGE_TURN";
    case FollowStatus::WILL_BE_LOST:      return "FollowStatus WILL_BE_LOST";
    case FollowStatus::LOST:               return "FollowStatus LOST";
    case FollowStatus::LOST_PREDICTING:   return "FollowStatus LOST_PREDICTING";
    case FollowStatus::LOST_BELIEF_NAV:   return "FollowStatus LOST_BELIEF_NAV";
    case FollowStatus::LOST_BELIEF_NAV_HISTORY: return "FollowStatus LOST_BELIEF_NAV_HISTORY";
    case FollowStatus::LOST_BELIEF_SCAN:  return "FollowStatus LOST_BELIEF_SCAN";
    case FollowStatus::LOST_HOLDING:      return "FollowStatus LOST_HOLDING";
    case FollowStatus::LOST_REWIND:       return "FollowStatus LOST_REWIND";
  }
  return "UNKNOWN";
}

void PersonFollowingNode::setFollowStatus(FollowStatus s)
{
  // Dedupe: only publish on change. Late subscribers still get the last value
  // because the publisher uses transient_local (latched) QoS.
  if (follow_status_ == s) {
    return;
  }
  follow_status_ = s;
  if (!status_pub_) {
    return;
  }
  std_msgs::msg::String msg;
  // Append the tracked id so consumers can correlate the status with the active
  // target. WILL_BE_LOST carries the id of the target that just went missing.
  std::string str = followStatusToStr(s);
  if (s == FollowStatus::TRACKING || s == FollowStatus::TRACKING_TOO_CLOSE ||
    s == FollowStatus::TRACKING_EDGE_TURN ||
    s == FollowStatus::IDLE_OBSERVING || s == FollowStatus::WILL_BE_LOST ||
    s == FollowStatus::LOST || s == FollowStatus::LOST_PREDICTING) {
    str += ":id=" + std::to_string(tracking_track_id_);
  }
  // Append the belief-search round to the belief sub-states so consumers can
  // see search progress (otherwise the status would sit on the same sub-state
  // for the whole belief search with no visible progress).
  if ((s == FollowStatus::LOST_BELIEF_NAV ||
       s == FollowStatus::LOST_BELIEF_NAV_HISTORY ||
       s == FollowStatus::LOST_BELIEF_SCAN) &&
    belief_search_.active) {
    str += ":round=" + std::to_string(belief_search_.round);
  }
  // Append the robot↔nav-goal distance to the belief-search NAV sub-states so
  // consumers can see how far the robot still has to go to the observation point.
  // last_nav_goal_dist_ is cached just before these statuses are published.
  if (s == FollowStatus::LOST_BELIEF_NAV || s == FollowStatus::LOST_BELIEF_NAV_HISTORY) {
    char gdist_buf[16];
    std::snprintf(gdist_buf, sizeof(gdist_buf), ":dist=%.2f", last_nav_goal_dist_);
    str += gdist_buf;
  }
  // Append the robot↔LKP distance to the belief-search SCAN sub-state so consumers
  // can see how far the robot is from the last-known-position while scanning.
  // Computed live from getCurrentPose() (no cache: scan can dwell in one state for
  // many frames and the cached pose would go stale; also compute is cheap).
  if (s == FollowStatus::LOST_BELIEF_SCAN) {
    auto robot_pose = getCurrentPose();
    char gdist_buf[16];
    double d = std::hypot(
      robot_pose.pose.position.x - belief_search_.lkp.x,
      robot_pose.pose.position.y - belief_search_.lkp.y);
    std::snprintf(gdist_buf, sizeof(gdist_buf), ":dist=%.2f", d);
    str += gdist_buf;
  }
  // Append the cached target distance for TRACKING / WILL_BE_LOST states so the
  // status-change frame carries :dist= consistently with the per-frame reports
  // published in publishGoalPose(). Uses the map-frame Euclidean distance (same
  // value the follow-distance band decides on), cached as last_target_map_dist_.
  // For WILL_BE_LOST this is the distance to where the target was last seen (the
  // last tracking frame's value). It may lag by one frame on the switching frame
  // (it is updated inside publishGoalPose, which runs after some callers invoke
  // setFollowStatus); subsequent per-frame reports are authoritative.
  if (s == FollowStatus::TRACKING || s == FollowStatus::TRACKING_TOO_CLOSE ||
    s == FollowStatus::TRACKING_EDGE_TURN ||
    s == FollowStatus::WILL_BE_LOST) {
    char dist_buf[16];
    std::snprintf(dist_buf, sizeof(dist_buf), ":dist=%.2f", last_target_map_dist_);
    str += dist_buf;
  }
  msg.data = str;
  status_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "follow status: %s", msg.data.c_str());
}

// 粗状态转换入口（去重）：在 IDLE→TRACKING（锁定/重锁）发 pattern 1（1 声），
// TRACKING→LOST（丢失）发 pattern 2（2 声）；→IDLE 静默。复用 originbot_base
// 的 /buzzer_pattern 声音库，本节点只决定"何时响、响几声"。
// 不 hook 在细粒度 follow_status_ 上——跟踪中子状态(TRACKING_TOO_CLOSE / 各种
// LOST_* 恢复策略)来回切会反复响；hook 在 3 态粗状态转换上才干净。
void PersonFollowingNode::setTrackState(TrackState s)
{
  if (track_state_ == s) {
    return;  // 去重：同态切换（如换目标 id 仍 TRACKING）不响
  }
  TrackState prev = track_state_;
  track_state_ = s;
  if (!buzzer_pattern_pub_) {
    return;
  }
  // 只在 TRACKING -> LOST 时响一声（pattern 1 = 1 短声）；进入 TRACKING（锁定/重锁）不响，
  // 避免锁定/重锁频繁发声。buzzer_min_interval_sec_ 在 publishBuzzerPattern 内进一步节流。
  if (s == TrackState::LOST && prev == TrackState::TRACKING) {
    publishBuzzerPattern(1);
  }
  // * -> TRACKING / -> IDLE：静默
}

void PersonFollowingNode::publishBuzzerPattern(uint8_t pattern)
{
  if (!buzzer_pattern_pub_) {
    return;
  }
  // buzzer_min_interval_sec_: <0 完全禁用（不发任何蜂鸣器消息）；==0 不限制；
  // >0 距上次发声不足该值则跳过（压 TRACKING↔LOST 闪烁震荡，避免持续嗡鸣）。
  if (buzzer_min_interval_sec_ < 0.0) {
    return;  // 禁用蜂鸣器
  }
  if (buzzer_min_interval_sec_ > 0.0 && buzzer_ever_fired_) {
    const double elapsed = (this->now() - last_buzzer_time_).seconds();
    if (elapsed < buzzer_min_interval_sec_) {
      RCLCPP_DEBUG(this->get_logger(),
        "buzzer pattern %d throttled (%.2fs < %.2fs min interval)",
        pattern, elapsed, buzzer_min_interval_sec_);
      return;
    }
  }
  last_buzzer_time_ = this->now();
  buzzer_ever_fired_ = true;
  std_msgs::msg::UInt8 msg;
  msg.data = pattern;
  buzzer_pattern_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "buzzer pattern %d on TRACKING->LOST", pattern);
}

void PersonFollowingNode::asyncNavToGoal(const NavigateToPose::Goal & goal)
{
  // Wait for the Nav2 action server; abort if unavailable.
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server not available after waiting");
    return;
  }

  RCLCPP_DEBUG(this->get_logger(), "Sending NavigateToPose goal at (%.2f, %.2f)",
    goal.pose.pose.position.x, goal.pose.pose.position.y);

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  // Register callbacks for goal acceptance/rejection and final result.
  send_goal_options.goal_response_callback =
    std::bind(&PersonFollowingNode::goalResponseCallback, this, _1);
  send_goal_options.result_callback =
    std::bind(&PersonFollowingNode::resultCallback, this, _1);
  goal_pending_ = true;  // set before send; cleared in goalResponseCallback
  nav_client_->async_send_goal(goal, send_goal_options);
}

void PersonFollowingNode::goalResponseCallback(
  GoalHandleNavigateToPose::SharedPtr goal_handle)
{
  goal_pending_ = false;  // response arrived — no longer awaiting accept/reject
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "NavigateToPose goal was rejected by server");
  } else {
    // Store the accepted goal handle so we can cancel it later if the target moves.
    nav_goal_handle_ = goal_handle;
    RCLCPP_DEBUG(this->get_logger(), "NavigateToPose goal accepted");
  }
}

void PersonFollowingNode::resultCallback(
  const GoalHandleNavigateToPose::WrappedResult & result)
{
  // Ignore stale results from goals that are no longer the active one. This
  // happens when a goal is canceled (e.g. FLOW 3 canceling the stale TRACKING
  // nav on entering LOST) and its delayed CANCELED result arrives after a new
  // goal (e.g. the belief-search observation point nav) has already been sent
  // and accepted — without this guard, the stale result would be mistaken for
  // the new goal arriving and trigger continueBeliefSearch prematurely (scanning
  // before actually reaching the observation point).
  // Ignore when:
  //  - a newer goal is pending (sent but not yet accepted): the result can't be
  //    for the newer goal, so it must be stale; or
  //  - the active handle is set and differs from this result's goal_id: stale.
  if (goal_pending_ ||
      (nav_goal_handle_ && result.goal_id != nav_goal_handle_->get_goal_id())) {
    RCLCPP_DEBUG(this->get_logger(),
      "Ignoring stale nav result (pending newer goal or goal_id mismatch)");
    return;
  }
  // Clear the active goal handle regardless of outcome.
  nav_goal_handle_ = nullptr;
  goal_pending_ = false;  // safety: a result always means accept already happened
  last_nav_goal_pose_ = nullptr;  // G13 should not suppress new goals vs a completed one
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_DEBUG(this->get_logger(), "NavigateToPose goal succeeded");
      // Proactive search: if still LOST on nav arrival, continue belief-guided
      // search — scan, then iterate to next observation point or end. When
      // belief search is inactive (it never started: seg1/seg3 staying-put),
      // there is nothing to continue and no active search to advance — a nav
      // result here is a stale/cancel residual, and there is no prediction
      // fallback anymore (it rarely triggered and was near-certain to fail);
      // just let the LOST timeout (lost_to_idle_timeout_sec_) → IDLE spin recover.
      if (track_state_ == TrackState::LOST) {
        if (belief_search_.active) {
          continueBeliefSearch();
        }
      }
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_INFO(this->get_logger(), "NavigateToPose goal was aborted");
      // Nav to an observation point failed (unreachable goal etc.). Don't stall
      // the belief search: advance it the same way as SUCCEEDED so it either
      // moves to the next observation point or ends. Otherwise (with the
      // LOST-timeout-suspended-while-active change) the search would sit idle
      // until belief_search_timeout_sec_ instead of progressing. When belief
      // search is inactive, there is nothing to advance (see SUCCEEDED note).
      if (track_state_ == TrackState::LOST) {
        if (belief_search_.active) {
          continueBeliefSearch();
        }
      }
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(this->get_logger(), "NavigateToPose goal was canceled");
      // Canceled goals are typically our own doing (cancel on target move /
      // re-entering LOST / IDLE). Only advance the search if still LOST and the
      // cancel wasn't because the search itself was torn down (active=false).
      if (track_state_ == TrackState::LOST && belief_search_.active) {
        continueBeliefSearch();
      }
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "NavigateToPose got unknown result code");
      break;
  }
}

void PersonFollowingNode::SpinInPlaceWCmdVel(float spin_radian) {
  // Blocking in-place spin using cmd_vel. Publishes angular velocity at a fixed
  // rate until the desired rotation angle is reached or a stop is requested.
  // WARNING: This blocks the executor thread; no other callbacks run during spin.
  if (spin_radian == 0.0f) {
    RCLCPP_WARN(this->get_logger(), "Spin InPlaceWCmdVel: spin_radian is 0, skipping");
    return;
  }

  spin_stop_requested_ = false;

  double spin_duration = std::abs(spin_radian) / explore_spin_angular_speed_;
  double angular_z = (spin_radian > 0) ? explore_spin_angular_speed_ : -explore_spin_angular_speed_;

  RCLCPP_DEBUG(this->get_logger(),
    "Spin InPlaceWCmdVel: spinning %.2f rad at %.2f rad/s, duration %.2f s, topic [%s]",
    spin_radian, angular_z, spin_duration, cmd_vel_topic_.c_str());

  geometry_msgs::msg::Twist twist;
  twist.angular.z = angular_z;

  rclcpp::Time start_time = this->now();
  const int publish_rate_hz = 10;
  rclcpp::Rate rate(publish_rate_hz);

  while (rclcpp::ok() && !spin_stop_requested_) {
    double elapsed = (this->now() - start_time).seconds();
    if (elapsed >= spin_duration) {
      break;
    }
    cmd_vel_pub_->publish(twist);
    rate.sleep();
  }

  // Publish zero velocity to guarantee the robot stops.
  geometry_msgs::msg::Twist stop_twist;
  cmd_vel_pub_->publish(stop_twist);
  spin_stop_requested_ = true;
  RCLCPP_DEBUG(this->get_logger(), "Spin InPlaceWCmdVel: completed, stopped=%s",
    spin_stop_requested_.load() ? "true" : "false");
}

}  // namespace tros_person_following

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(tros_person_following::PersonFollowingNode)
