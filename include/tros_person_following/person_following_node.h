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

#ifndef TROS_PERSON_FOLLOWING__PERSON_FOLLOWING_NODE_H_
#define TROS_PERSON_FOLLOWING__PERSON_FOLLOWING_NODE_H_

#include <string>
#include <map>
#include <vector>
#include <mutex>
#include <atomic>
#include <thread>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <ai_msgs/msg/perception_targets.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace tros_person_following
{
// Collect valid person targets with position info
struct PersonTarget {
  const ai_msgs::msg::Target * target;
  double x_m;
  double y_m;
  float height_m;
  float width_m;
  float confidence;        // person ROI confidence (0-1)
  float bbox_center_x;     // bbox center x in pixels
};

// One world-frame position sample of a detected person (for stationary-target
// detection and activity scoring).
struct TargetPositionSample {
  geometry_msgs::msg::Point world_pos;  // person position in map frame
  rclcpp::Time stamp;
};

class PersonFollowingNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit PersonFollowingNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PersonFollowingNode();

private:
  // Detection result callback — ported from frontier_exploration Explore::dectectResultCallback
  void detectResultCallback(const ai_msgs::msg::PerceptionTargets::SharedPtr msg);

  // /enable_follow service callback: true starts following, false stops.
  void enableFollowCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  // Start following: cancel any external nav goal, enter IDLE, arm spin search.
  void startFollowing();

  // Stop following: cancel current follow goal, clear speed limit, return to IDLE.
  void stopFollowing();

  // Navigate to a pose asynchronously
  void asyncNavToGoal(const NavigateToPose::Goal & goal);

  // Spin in place by the given radian using NavigateToPose action
  void spinInPlace(float spin_radian);

  // Utility: get pointing pose (face toward goal from base)
  geometry_msgs::msg::PoseStamped getPointingPose(
    const geometry_msgs::msg::PoseStamped & base,
    const geometry_msgs::msg::PoseStamped & goal);

  // Utility: get forward pose from base along its orientation
  geometry_msgs::msg::PoseStamped getForwardPose(
    const geometry_msgs::msg::PoseStamped & base,
    float forward_dist);

  // Utility: compute a trailing goal — a point `follow_distance` behind the
  // person along the robot→person line, oriented to face the person (map frame).
  geometry_msgs::msg::PoseStamped getTrailingPose(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::PoseStamped & person_pose,
    double follow_distance);

  // Goal = the person's position if it's free in the costmap; else step from the
  // person toward the robot until a free cell is found (closest-to-person free
  // point). Oriented along robot→person. Falls back to the person position if
  // no costmap / no free cell found.
  geometry_msgs::msg::PoseStamped getFreePersonGoal(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::PoseStamped & person_pose);
  // True if the costmap cell at `pt` (map frame) is free
  // (0 <= cost < costmap_free_cost_thr_). False if no costmap / out of bounds.
  bool isPointFree(const geometry_msgs::msg::Point & pt);
  // Map-frame world point → costmap grid indices (mx, my). False if cm is null,
  // resolution <= 0, or the point is outside the costmap. Factored from
  // isPointFree so the BFS/Bresenham helpers reuse one conversion path.
  bool worldToCell(const nav_msgs::msg::OccupancyGrid::SharedPtr & cm,
                   const geometry_msgs::msg::Point & pt, int & mx, int & my);
  // Raw costmap_2d cost at (mx, my) as int in [0, 255]; -1 if out of bounds.
  // Casts through uint8_t so LETHAL(254)/NO_INFORMATION(255) survive the
  // int8_t→int promotion (otherwise they read as -2/-1).
  int cellCost(const nav_msgs::msg::OccupancyGrid::SharedPtr & cm, int mx, int my);
  // BFS (8-neighbour, Chebyshev grid distance) for the nearest free cell to the
  // person's cell, bounded by max_radius_m. Free = 0 <= cost < costmap_free_cost_thr_.
  // On success fills `out` (cell centre, map frame) and returns true; false if no
  // costmap, person cell out of bounds, or no free cell within the radius.
  bool nearestFreeCellToPerson(const geometry_msgs::msg::Point & person_xy,
                               double max_radius_m, geometry_msgs::msg::Point & out);
  void globalCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  // Verify an arbitrary goal is reachable via ComputePathToPose. Returns true if
  // a path can be planned to it.
  bool isGoalReachable(const geometry_msgs::msg::PoseStamped & goal);

  // True if the person ROI center falls in the left/right edge band of the image.
  bool isTargetAtFrameEdge(const ai_msgs::msg::Target & target);

  // Transform a camera-frame person pose into the global (map) frame using the
  // given robot pose's frame_id. Returns PoseStamped with empty frame_id on TF fail.
  geometry_msgs::msg::PoseStamped transformPersonToWorld(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::PoseStamped & pose_camera);
  // World-frame distance moved by `id` over the last `window_sec` seconds.
  // Returns -1.0 if fewer than 2 samples in window.
  double targetMovementOverWindow(uint64_t id, double window_sec);
  // Erase samples older than max_age_sec across all ids.
  void pruneTargetPosHistory(double max_age_sec);
  // Predict the world-frame pose of person `id` from recent motion history.
  // Returns PoseStamped with empty header.frame_id when no usable prediction
  // (too few samples, stationary, or stale newest sample).
  geometry_msgs::msg::PoseStamped predictTargetPose(uint64_t id);
  // Publish the predicted trajectory of the tracked person: recent world-frame
  // history samples + the extrapolated predicted point (when available) as a
  // nav_msgs/Path. Called every frame while TRACKING for RViz visualization.
  void publishPredictTrajectory();
  // Publish the belief-guided search path for RViz visualization: LKP + already
  // searched observation points + the current target observation point, as a
  // nav_msgs/Path on "predict_trajectory". Reuses the same topic as
  // publishPredictTrajectory because the two states (TRACKING vs LOST belief
  // search) are mutually exclusive. Called on each observation-point selection
  // and on belief-search cleanup (empty path clears the display).
  void publishBeliefSearchPath();
  // Publish the followed target info (ai_msgs/PerceptionTargets on
  // "tros_person_followed"): track_id + rois + attributes of the currently
  // followed target, plus a follow_status attribute. Called on status change.
  void publishFollowedTarget();
  // Publish the followed target's map-frame pose as a PoseStamped on
  // "tros_followed_target_pose". Called every frame while tracking once the
  // camera-frame target pose has been transformed to the global frame.
  void publishFollowedTargetPose(const geometry_msgs::msg::PoseStamped & target_pose);
  // Multi-factor target selection score: weighted sum of distance, confidence,
  // and image-center factors. Higher = better target to follow.
  float computeTargetScore(const PersonTarget & p);
  // --- Belief-Guided search (Improvement M) ---
  // Extract last known position + velocity from history (no stale filter).
  bool getLastKnownPose(uint64_t id, geometry_msgs::msg::Point & lkp,
    double & vx, double & vy, double & speed);
  // Resolve the LKP (last-known position) to use as a re-lock distance anchor.
  // Prefers the belief search's LKP (which may be rewound from out-of-costmap
  // history), falling back to the freshest history sample when belief search
  // is inactive (e.g. after the pending-lost window without an active search).
  bool getRelockLkp(geometry_msgs::msg::Point & lkp) const;
  // Select the next observation point based on LKP + motion direction + costmap.
  // Returns empty frame_id if no suitable free cell found. `from_history` is set
  // true when the returned point was rewound from the target's history trajectory
  // (vs an LKP-area cell), so the caller can report a distinct status.
  geometry_msgs::msg::PoseStamped pickLkpObservationPose(bool & from_history);
  // Start belief-guided search: extract LKP/velocity, pick first observation point, navigate.
  void startBeliefSearch();
  // Continue belief-guided search after nav arrival: scan, then iterate or end.
  void continueBeliefSearch();

  // Get current robot pose in the global frame
  geometry_msgs::msg::PoseStamped getCurrentPose();

  // Publish goal pose for a tracked target
  void publishGoalPose(
    const ai_msgs::msg::Target & target,
    const builtin_interfaces::msg::Time & stamp);

  // Publish enable_blind_zone_observing state
  void publishBlindZoneObserving(bool enable);

  // Follow status reporting on "tros_tracking_status" (std_msgs/String, latched).
  // Layered on top of TrackState purely for status publishing — does not drive
  // control flow. Mirrors frontier_exploration's tros_tracking_status pattern.
  enum class FollowStatus {
    DISABLED,            // follow_enabled_ == false
    IDLE_SEARCHING,      // IDLE, searching/spinning for a person
    IDLE_OBSERVING,      // IDLE, paused to observe a candidate target (no spin)
    TRACKING,           // actively tracking/following
    TRACKING_TOO_CLOSE, // tracking but person < follow_min_safe_distance_, goals withheld
    TRACKING_EDGE_TURN, // tracking, person at frame edge, in-place cmd_vel spin toward target (no Nav2)
    WILL_BE_LOST,       // target not detected this frame, in the pending-lost window (tracking_to_lost_timeout_sec_) before transitioning to LOST
    LOST,                // target acquired then lost
    LOST_PREDICTING,     // LOST and proactively navigating to a predicted target position
    LOST_BELIEF_NAV,     // LOST, belief search navigating to an observation point (LKP-area)
    LOST_BELIEF_NAV_HISTORY, // LOST, belief search navigating to a free point rewound from history (not LKP-area)
    LOST_BELIEF_SCAN,    // LOST, belief search arrived at a point, in-place directional scan running
    LOST_HOLDING,        // LOST, staying put (no motion history / no free observation point), waiting LOST timeout -> IDLE
    LOST_REWIND          // LOST, rewinding history to an in-bounds sample as search center (LKP was out of costmap)
  };
  FollowStatus follow_status_{FollowStatus::DISABLED};
  void setFollowStatus(FollowStatus s);
  std::string followStatusToStr(FollowStatus s);

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ======================================================================
  // Topics & Frames — 话题与坐标系
  // ======================================================================
  std::string detect_result_topic_name_;    // 订阅感知结果的话题（ai_msgs/PerceptionTargets）
  std::string goal_pose_topic_name_ = "nearest_pose";      // 发布目标位姿的话题（调试/可视化）
  std::string global_frame_ = "map";        // 全局坐标系（Nav2 goal / 距离计算）
  std::string robot_frame_ = "base_footprint";  // 机器人坐标系
  std::string followed_target_pose_topic_ = "tros_followed_target_pose";  // 跟踪目标 map 系位姿话题
  std::string followed_target_topic_ = "tros_person_followed";  // 跟踪目标信息话题（ai_msgs）
  std::string status_topic_ = "tros_tracking_status";  // 跟随状态话题（std_msgs/String，latched）
  std::string predict_traj_topic_ = "predict_trajectory";  // 预测轨迹可视化话题（nav_msgs/Path）
  std::string cmd_vel_topic_ = "/cmd_vel";  // 原地旋转发 cmd_vel 的话题
  std::string costmap_topic_ = "/global_costmap/costmap";  // 全局 costmap 订阅话题

  // ======================================================================
  // Target filtering — IDLE 期目标过滤（丢弃不达标目标）
  // ======================================================================
  float target_filter_height_thr_ = 0.5f;       // 最小人体高度（m），低于此值视为误检
  float target_filter_width_thr_ = 0.3f;        // 最小人体宽度（m），低于此值视为误检
  float target_filter_range_x_min_ = 0.1f;      // 有效范围 X 最小值（m，相机前方）
  float target_filter_range_x_max_ = 4.0f;      // 有效范围 X 最大值（m）
  float target_filter_range_y_min_ = -3.0f;     // 有效范围 Y 最小值（m，相机左侧）
  float target_filter_range_y_max_ = 3.0f;      // 有效范围 Y 最大值（m，相机右侧）
  float target_filter_confidence_thr_ = 0.5f;   // 最小置信度，低于此值在 IDLE 状态不跟踪（垃圾过滤）

  // ======================================================================
  // Target selection — IDLE 期目标选择（在达标目标里挑最佳）
  // ======================================================================
  // 比 target_filter_confidence_thr_ 更严苛的"质量门槛"：moving 候选 conf 低于此值不选为跟随目标
  // （继续 spin 找高质量目标）。高于 target_filter_confidence_thr_（后者只滤垃圾）。
  double select_min_confidence_ = 0.7;
  // 多因子打分权重（dist+conf+center，和应≈1.0）：选 moving 候选里分最高的跟随。
  float select_weight_dist_ = 0.5f;     // 距离因子权重（越近分越高）
  float select_weight_score_ = 0.3f;   // 置信度因子权重
  float select_weight_center_ = 0.2f;   // 画面居中度因子权重

  // ======================================================================
  // Follow distance band — 跟随距离带（停车 / withhold / 跟随）
  // ======================================================================
  double follow_distance_min_ = 1.8;   // dist<min：停车（cancel nav + 零速 + edge-turn 条件）
  double follow_distance_max_ = 2.0;   // dist>max：跟随（发 NavigateToPose）；中间为 withhold
  // 带 max 边界迟滞：进 follow 需 dist>max+hyst，退 follow 需 dist<max-hyst，
  // 防 dist 在 max 附近抖动反复 send/cancel nav。following_active_ 保存粘滞状态
  // （true=在 follow，false=在 withhold），每次"开始跟新目标"必须重置为 false，
  // 否则新目标首帧会跳过 enter 阈值直接 follow。
  double follow_hysteresis_ = 0.1;
  bool following_active_ = false;
  double follow_min_safe_distance_ = 0.5;    // 相机 X 方向硬停距离（m），低于此值停车
  double follow_goal_pub_rate_ = 2.5;         // Nav2 goal 重发频率上限（Hz）
  // Nav goal deadzone：goal 与当前位姿/上次 goal 的位移差 < dist_thr 且角度差 < yaw_thr 时不重发
  // （防小幅 goal 变化反复触发 nav）。
  float follow_goal_dist_deadzone_ = 0.3f;     // 位移死区（m）
  float follow_goal_yaw_deadzone_ = 0.6;       // 角度死区（rad，约 34°）

  // ======================================================================
  // Edge-of-frame turn — 目标在画面边缘时原地转向挽留
  // ======================================================================
  double image_width_ = 640.0;         // 相机图像宽度（px，ROI 边缘判定）
  double edge_margin_ratio_ = 0.15;    // 左右边缘带占比（×image_width）

  // ======================================================================
  // Stationary-target switch — 静止目标切换（跟的人不动，切到附近活动的目标）
  // ======================================================================
  double static_target_timeout_sec_ = 5.0;   // 跟踪目标静止超此时长（s）→ 可切换
  double static_target_move_thr_ = 0.1;      // 世界位移低于此值（m）= 静止
  double static_switch_depth_diff_thr_ = 0.3;       // |A.x - B.x| 低于此值（m）= 深度相近
  double static_switch_activity_window_sec_ = 1.0;  // 测量候选 B 活动度的窗口（s）
  // 各 track_id 的世界系位置历史（用于 A 静止 + B 活动判定 + LKP/预测）。
  std::map<uint64_t, std::vector<TargetPositionSample>> target_pos_history_;

  // ======================================================================
  // Prediction — LOST 后轨迹预测（速度外推找目标未来位置）
  // ======================================================================
  double predict_window_sec_ = 6.0;    // 速度估计窗口（s）
  double predict_lead_sec_ = 2.0;      // 前瞻外推时长（s，外推 v×(since+lead)）
  double predict_max_dist_ = 3.0;       // 外推位移上限（m，超出按比例缩放）
  double predict_stale_sec_ = 10.0;      // 最新样本超此年龄（s）则不预测

  // ======================================================================
  // IDLE search & observe — IDLE 搜索 / 观察 / 原地旋转
  // ======================================================================
  float idle_search_start_timeout_sec_ = 3.0f;  // IDLE 多久无目标开始旋转搜索（s）
  float idle_search_total_timeout_sec_ = 60.0f;      // IDLE 搜索总时长上限（s，超时停 spin）
  double idle_observe_duration_sec_ = 1.0; // 观察 valid-but-not-moving 候选的时长（s）
  double observe_cooldown_sec_ = 3.0;      // 观察超时后跳过再观察的冷却（s），让 spin 真转起来
  float spin_radian_ = 3.14f;         // 每次原地旋转角度（rad）
  double explore_spin_angular_speed_ = 0.8; // 原地旋转角速度（rad/s）

  // ======================================================================
  // LOST recovery / belief search / relock — LOST 恢复 + 信念搜索 + 重锁
  // ======================================================================
  float tracking_to_lost_timeout_sec_ = 2.0f;  // TRACKING 连续丢失超此时长（s）→ 进 LOST
  float lost_to_idle_timeout_sec_ = 1.0f;      // LOST 持续超此时长（s，belief 未激活时）→ IDLE
  double belief_search_timeout_sec_ = 8.0;  // belief search 总超时（s）
  int belief_search_max_rounds_ = 2;         // belief search 最大迭代轮数
  // LOST 持续时间分段的 relock 距离阈值（早期严格、晚期宽松）：
  // FLOW 7（找回原 id）/ FLOW 8（接受新目标）只接受距 LKP 在当前阈值内的目标，
  // 防止锁错无关路人。阈值随 LOST 持续时间分段放宽：phase1（刚丢，严）→ phase2 → phase3（久丢，宽）。
  double relock_dist_phase1_ = 1.0;
  double relock_dist_phase2_ = 2.0;
  double relock_dist_phase3_ = 3.0;
  double relock_dist_phase1_sec_ = 2.0;   // phase1→phase2 切换点（s）
  double relock_dist_phase2_sec_ = 5.0;   // phase2→phase3 切换点（s）
  double currentRelockDist() const;       // 按 LOST 持续时间返回当前阶段阈值

  // ======================================================================
  // Nav2 / cmd_vel — Nav2 导航客户端 + 速度发布
  // ======================================================================
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  GoalHandleNavigateToPose::SharedPtr nav_goal_handle_ = nullptr;
  // goal 已发但 accept/reject 未回期间为 true（goalResponseCallback 清除）。
  // 此时 nav_goal_handle_ 仍为 null，!nav_goal_handle_ 会误判无在飞 goal，
  // 加 !goal_pending_ 防重复发（如 edge-turn 与 goal-accept 回调竞态）。
  std::atomic<bool> goal_pending_{false};
  std::shared_ptr<geometry_msgs::msg::PoseStamped> last_nav_goal_pose_ = nullptr;
  rclcpp::Time last_goal_send_time_;
  bool goal_send_armed_{false};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_ = nullptr;
  void goalResponseCallback(GoalHandleNavigateToPose::SharedPtr goal_handle);
  void resultCallback(const GoalHandleNavigateToPose::WrappedResult & result);

  // ======================================================================
  // Costmap — 全局 costmap 订阅 + free 判定（放 nav goal 时用）
  // ======================================================================
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
  std::mutex costmap_mutex_;
  double costmap_free_cost_thr_ = 50.0;       // cell cost 低于此值视为 free
  double costmap_free_search_radius_m_ = 2.0; // 人附近 BFS 最近 free cell 的半径（m）

  // ======================================================================
  // Tracking state — 状态机 + 跟踪 id + 关键时间戳
  // ======================================================================
  enum class TrackState { IDLE, TRACKING, LOST };
  TrackState track_state_{TrackState::IDLE};
  uint64_t tracking_track_id_{0};     // 当前跟随目标 track_id（IDLE 时为 0）
  bool target_lost_{false};           // TRACKING 期本帧未检测到目标（pending-lost 宽限期内）
  rclcpp::Time tp_target_lost_;       // 首次丢失时刻（进 pending-lost 宽限期）
  rclcpp::Time tp_lost_;             // 进 LOST 时刻（relock timeline 起点）
  rclcpp::Time tp_target_find_start_;  // IDLE 搜索开始时刻（计 idle_search_total_timeout_sec）
  uint64_t observe_track_id_{0};     // IDLE 观察期候选 track_id
  rclcpp::Time tp_observe_start_;    // IDLE 观察期开始时刻
  rclcpp::Time tp_observe_timeout_;  // 上次观察超时时刻（0=从未，cooldown 用）
  // 旋转方向（+1 左 / -1 右）：目标消失时记录其相对机器人前向的侧，搜索 spin 朝该侧转。
  float last_target_direction_ = 1.0f;

  // ======================================================================
  // Spin machinery — 原地旋转线程（IDLE 搜索 / belief scan / edge-turn 共用）
  // ======================================================================
  void SpinInPlaceWCmdVel(float spin_radian);
  std::atomic<bool> spin_stop_requested_{false};  // 请求停止旋转（线程内循环检查）
  // SpinInPlaceWCmdVel 在 detached 线程跑（不阻塞 detectResultCallback，避免漏检）。
  std::atomic<bool> spin_active_{false};
  std::shared_ptr<std::thread> spin_thread_;

  // ======================================================================
  // Status & diagnostics — 状态发布 + 距离缓存（追加到状态串 :dist=）
  // ======================================================================
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_ = nullptr;
  // 上一帧相机系前向深度（m）：setFollowStatus 去重，状态切换帧需带 :dist=。
  double last_target_dist_ = -1.0;
  // 上一帧 map 系欧氏距离（m）：跟随距离带判断用的同一值，发布到 :dist=。
  double last_target_map_dist_ = -1.0;
  // 上一帧 robot↔nav-goal（观测点）距离（m）：LOST_BELIEF_NAV 的 :dist=。
  double last_nav_goal_dist_ = -1.0;
  // 缓存最新检测帧，供 publishFollowedTarget 用。
  ai_msgs::msg::PerceptionTargets latest_detection_;

  // ======================================================================
  // Subscribers & Publishers — 订阅与发布
  // ======================================================================
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr detect_result_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr followed_target_pose_pub_ = nullptr;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_blind_zone_observing_pub_;
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr followed_target_pub_ = nullptr;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predict_traj_pub_ = nullptr;
  // /enable_follow service：门控所有跟踪行为。
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_follow_srv_;
  bool follow_enabled_{false};

  // ======================================================================
  // Belief search state — belief search 运行期状态（结构体）
  // ======================================================================
  struct BeliefSearchState {
    bool active = false;
    int round = 0;                          // 当前迭代（0-based）
    int max_rounds = 2;                     // 最大迭代数
    geometry_msgs::msg::Point lkp;         // 最后已知位置（map 系）
    double vx = 0.0, vy = 0.0;             // 丢失时运动方向
    double speed = 0.0;                     // 丢失时速度
    bool has_motion = false;               // speed >= static_target_move_thr_ 则 true
    std::vector<geometry_msgs::msg::Point> searched_points;  // 已访观测点（下轮排除）
    rclcpp::Time start_time;                // belief search 开始时刻（计 timeout）
    bool scanning = false;                  // 到达后原地扫描中
    bool scan_completed = false;            // 本观测点扫描完成（区别于 scanning==false 的"未开始"）
    bool at_lkp = false;                    // 观测点即 LKP/预测点（→ 到达即扫）；false 为过渡 free cell（→ 不扫，下轮重试 LKP）
  } belief_search_;


};

}  // namespace tros_person_following

#endif  // TROS_PERSON_FOLLOWING__PERSON_FOLLOWING_NODE_H_
