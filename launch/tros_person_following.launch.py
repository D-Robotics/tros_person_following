# Copyright (c) 2024，D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_respawn = LaunchConfiguration('use_respawn')
    container_name = LaunchConfiguration('container_name')
    log_level = LaunchConfiguration('log_level')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to use re-spawning of processes')
    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='tros_container',
        description='the name of the container that launches all nodes')
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='log level')
    declare_depth_fusion_detect_input_width_cmd = DeclareLaunchArgument(
        'depth_fusion_detect_input_width',
        default_value='640',
        description='the width of the input image of detection')
    declare_depth_fusion_detect_input_height_cmd = DeclareLaunchArgument(
        'depth_fusion_detect_input_height',
        default_value='352',
        description='the height of the input image of detection')

    # tros_person_following launch arguments
    declare_detect_result_topic_cmd = DeclareLaunchArgument(
        'detect_result_topic_name',
        default_value='/tros_mot_targets',
        description='Detection result topic name for robot control')
    declare_goal_pose_topic_cmd = DeclareLaunchArgument(
        'goal_pose_topic_name',
        default_value='nearest_pose',
        description='Goal pose topic name')
    declare_global_frame_cmd = DeclareLaunchArgument(
        'global_frame',
        default_value='map',
        description='Global frame')
    declare_robot_frame_cmd = DeclareLaunchArgument(
        'robot_frame',
        default_value='base_footprint',
        description='Robot frame')
    declare_person_track_timeout_cmd = DeclareLaunchArgument(
        'idle_search_start_timeout_sec',
        default_value='3.0',
        description='Person track timeout in seconds')
    declare_spin_radian_cmd = DeclareLaunchArgument(
        'spin_radian',
        default_value='3.14',
        description='Spin radian for in-place rotation')
    declare_explore_spin_angular_speed_cmd = DeclareLaunchArgument(
        'explore_spin_angular_speed',
        default_value='0.8',
        description='Angular speed (rad/s) for in-place spin (IDLE search, belief scan, edge-turn)')
    declare_target_find_timeout_cmd = DeclareLaunchArgument(
        'idle_search_total_timeout_sec',
        default_value='60.0',
        description='Timeout in seconds for searching targets before stopping spin')
    declare_idle_observe_duration_sec_cmd = DeclareLaunchArgument(
        'idle_observe_duration_sec',
        default_value='1.0',
        description='Pause-and-observe window (s) when a valid but not-yet-moving target appears during search')
    declare_observe_cooldown_sec_cmd = DeclareLaunchArgument(
        'observe_cooldown_sec',
        default_value='3.0',
        description='After an observe-timeout, skip re-observing any stationary target for this long (s) so the IDLE spin can turn')
    declare_timeout_sec_lost2idle_cmd = DeclareLaunchArgument(
        'lost_to_idle_timeout_sec',
        default_value='1.0',
        description='Lost state timeout in seconds')
    declare_timeout_sec_tracking2lost_cmd = DeclareLaunchArgument(
        'tracking_to_lost_timeout_sec',
        default_value='2.0',
        description='Continuous lost duration threshold before transitioning to LOST state in seconds')
    declare_width_thr_cmd = DeclareLaunchArgument(
        'target_filter_width_thr',
        default_value='0.3',
        description='Minimum person width threshold in meters')
    declare_height_thr_cmd = DeclareLaunchArgument(
        'target_filter_height_thr',
        default_value='0.5',
        description='Minimum person height threshold in meters')
    declare_range_x_min_cmd = DeclareLaunchArgument(
        'target_filter_range_x_min',
        default_value='0.1',
        description='Minimum x range in meters')
    declare_range_x_max_cmd = DeclareLaunchArgument(
        'target_filter_range_x_max',
        default_value='4.0',
        description='Maximum x range in meters')
    declare_range_y_min_cmd = DeclareLaunchArgument(
        'target_filter_range_y_min',
        default_value='-3.0',
        description='Minimum y range in meters')
    declare_range_y_max_cmd = DeclareLaunchArgument(
        'target_filter_range_y_max',
        default_value='3.0',
        description='Maximum y range in meters')
    declare_nav_goal_yaw_thr_cmd = DeclareLaunchArgument(
        'follow_goal_yaw_deadzone',
        default_value='0.6',
        description='Yaw threshold in radians to trigger navigation (~34 degrees)')
    declare_follow_distance_min_cmd = DeclareLaunchArgument(
        'follow_distance_min',
        default_value='1.8',
        description='Map-distance (m) below which following stops and the nav goal is canceled')
    declare_follow_distance_max_cmd = DeclareLaunchArgument(
        'follow_distance_max',
        default_value='2.0',
        description='Map-distance (m) above which following starts (nav goal = person position)')
    declare_follow_hysteresis_cmd = DeclareLaunchArgument(
        'follow_hysteresis',
        default_value='0.1',
        description='Hysteresis (m) around follow_distance_max: enter follow at dist>max+hyst, exit at dist<max-hyst (prevents send/cancel nav churn at the band edge)')
    declare_static_target_timeout_sec_cmd = DeclareLaunchArgument(
        'static_target_timeout_sec',
        default_value='5.0',
        description='Tracked person stationary this long (s) -> eligible to switch to an active target')
    declare_static_target_move_thr_cmd = DeclareLaunchArgument(
        'static_target_move_thr',
        default_value='0.1',
        description='World-frame movement (m) below which the tracked person counts as stationary')
    declare_switch_depth_diff_thr_cmd = DeclareLaunchArgument(
        'static_switch_depth_diff_thr',
        default_value='0.3',
        description='|A.x - B.x| (m) below which candidate B is at a close depth to A')
    declare_switch_activity_window_sec_cmd = DeclareLaunchArgument(
        'static_switch_activity_window_sec',
        default_value='1.0',
        description='Window (s) over which a candidate B movement is measured')
    declare_predict_window_sec_cmd = DeclareLaunchArgument(
        'predict_window_sec',
        default_value='6.0',
        description='Window (s) for estimating the tracked person velocity when predicting on LOST')
    declare_predict_lead_sec_cmd = DeclareLaunchArgument(
        'predict_lead_sec',
        default_value='2.0',
        description='Extrapolate the predicted position this far ahead beyond now (s)')
    declare_predict_max_dist_cmd = DeclareLaunchArgument(
        'predict_max_dist',
        default_value='3.0',
        description='Clamp the predicted displacement from last-seen position (m)')
    declare_predict_stale_sec_cmd = DeclareLaunchArgument(
        'predict_stale_sec',
        default_value='10.0',
        description='Skip prediction if the newest motion sample is older than this (s)')
    declare_costmap_topic_cmd = DeclareLaunchArgument(
        'costmap_topic',
        default_value='/global_costmap/costmap',
        description='Global costmap topic (nav_msgs/OccupancyGrid) for free-cell checks')
    declare_followed_target_topic_cmd = DeclareLaunchArgument(
        'followed_target_topic',
        default_value='tros_person_followed',
        description='Topic name for publishing the followed target info (ai_msgs/PerceptionTargets)')
    declare_followed_target_pose_topic_cmd = DeclareLaunchArgument(
        'followed_target_pose_topic',
        default_value='tros_followed_target_pose',
        description='Topic name for publishing the followed target pose (geometry_msgs/PoseStamped) while tracking')
    declare_goal_free_cost_thr_cmd = DeclareLaunchArgument(
        'costmap_free_cost_thr',
        default_value='50.0',
        description='Costmap cell cost below this counts as free when placing the nav goal')
    # Belief-guided search (Improvement M)
    declare_belief_search_timeout_sec_cmd = DeclareLaunchArgument(
        'belief_search_timeout_sec',
        default_value='8.0',
        description='Max belief search duration (s) before falling back to IDLE spin')
    declare_belief_search_max_rounds_cmd = DeclareLaunchArgument(
        'belief_search_max_rounds',
        default_value='2',
        description='Max belief search iterations')
    declare_relock_dist_phase1_cmd = DeclareLaunchArgument(
        'relock_dist_phase1',
        default_value='1.0',
        description='Relock distance threshold (m) for LOST duration < relock_dist_phase1_sec (strict, just lost)')
    declare_relock_dist_phase2_cmd = DeclareLaunchArgument(
        'relock_dist_phase2',
        default_value='2.0',
        description='Relock distance threshold (m) for phase1_sec <= LOST duration < phase2_sec (medium)')
    declare_relock_dist_phase3_cmd = DeclareLaunchArgument(
        'relock_dist_phase3',
        default_value='3.0',
        description='Relock distance threshold (m) for LOST duration >= relock_dist_phase2_sec (lenient, lost long)')
    declare_relock_dist_phase1_sec_cmd = DeclareLaunchArgument(
        'relock_dist_phase1_sec',
        default_value='2.0',
        description='LOST duration (s) at which relock threshold switches phase1 -> phase2')
    declare_relock_dist_phase2_sec_cmd = DeclareLaunchArgument(
        'relock_dist_phase2_sec',
        default_value='5.0',
        description='LOST duration (s) at which relock threshold switches phase2 -> phase3')
    declare_confidence_thr_cmd = DeclareLaunchArgument(
        'target_filter_confidence_thr',
        default_value='0.5',
        description='Minimum confidence threshold for target selection in IDLE state')
    declare_select_min_confidence_cmd = DeclareLaunchArgument(
        'select_min_confidence',
        default_value='0.7',
        description='Stricter confidence gate for IDLE target selection: moving candidates below this are not picked (keep spinning). Higher than target_filter_confidence_thr.')
    # Multi-factor target selection weights
    declare_select_weight_dist_cmd = DeclareLaunchArgument(
        'select_weight_dist',
        default_value='0.5',
        description='Weight for distance factor in target selection')
    declare_select_weight_score_cmd = DeclareLaunchArgument(
        'select_weight_score',
        default_value='0.3',
        description='Weight for confidence factor in target selection')
    declare_select_weight_center_cmd = DeclareLaunchArgument(
        'select_weight_center',
        default_value='0.2',
        description='Weight for image-center factor in target selection')
    declare_edge_margin_ratio_cmd = DeclareLaunchArgument(
        'edge_margin_ratio',
        default_value='0.15',
        description='Fraction of image width counted as left/right edge band for edge-turn-to-target')
    declare_image_width_cmd = DeclareLaunchArgument(
        'image_width',
        default_value='640.0',
        description='Camera image width in px (double) for ROI edge check')
    declare_mot_sub_topic_cmd = DeclareLaunchArgument(
        'mot_sub_topic',
        default_value='/tros_fusion_interaction',
        description='MOT node subscription topic')
    declare_mot_pub_topic_cmd = DeclareLaunchArgument(
        'mot_pub_topic',
        default_value='/tros_mot_targets',
        description='MOT node publication topic')
    declare_mot_config_cmd = DeclareLaunchArgument(
        'mot_config',
        default_value=os.path.join(
            get_package_share_directory('tros_person_following'),
            'config', 'iou2_method_param.json'),
        description='MOT config json path (absolute). Default: tros_person_following share config.')
    declare_enable_perc_render_cmd = DeclareLaunchArgument(
        'enable_perc_render',
        default_value='False',
        description='Whether to launch perception render node')
    declare_enable_websocket_cmd = DeclareLaunchArgument(
        'enable_websocket',
        default_value='True',
        description='Whether to launch websocket node')
    declare_buzzer_min_interval_sec_cmd = DeclareLaunchArgument(
        'buzzer_min_interval_sec',
        default_value='3.0',
        description='Buzzer throttle (s): >0 min interval between beeps; ==0 no limit; <0 disable buzzer entirely. Only TRACKING->LOST beeps (1 short).')

    load_nodes = GroupAction(
        actions=
        [
            # fusion framework 节点：感知和深度的融合
            # obstacle_depth_fusion_node
            Node(
                package='hobot_obstacle_depth_fusion',
                executable='hobot_obstacle_depth_fusion',
                parameters=[
                        {'depth_msg_topic': '/StereoNetNode/stereonet_depth'},
                        {'seg_result_msg_topic': '/hobot_dnn_seg'},
                        {'pub_fusion_msg_topic_seg': '/tros_fusion_interaction'},
                        {'detect_mode': 2},
                        {'detect_input_width': LaunchConfiguration('depth_fusion_detect_input_width')},
                        {'detect_input_height': LaunchConfiguration('depth_fusion_detect_input_height')},
                        {'seg_output_width': LaunchConfiguration('depth_fusion_detect_input_width')},
                        {'seg_output_height': LaunchConfiguration('depth_fusion_detect_input_height')},
                        {'enable_pcl_cvt_seg': True},
                        {'point_cloud_target_frame': 'pcl_link'},
                        {'enable_pub_map': False},
                        {'camera_info_rect_topic': '/StereoNetNode/rectify_left_image/camera_info'},
                        {'stereo_blind_zone': 0.09}, # Depth Blind Spot Distance, unit is meter
                        {'stereo_blind_scale': 0.5},
                        {'max_obstacle_depth': 5.0}, # The maximum depth of the obstacle, the obstacles will be filtered if depth exceeds this threshold, unit is meter
                        {'depth_side_area': 0.1}, # Left/Right Depth Blind Spot Region ratio
                        {'depth_side_scale': 0.5},
                        {'depth_hight_threshold': 10.1}, # The region on the obstacle above this height will be filtered, unit is meter
                        {'depth_vaild_area': 1.0}, # The ratio of valid middle region, the perc and depth datas on left and right region will be filtered
                        {'seg_filter_min_area_ratio': 0.0005}, # 分割区域外接矩形的面积（单位：像素）占分割图像的比例最小值，用于过滤掉小面积的分割区域
                        {'seg_filter_max_area_ratio': 0.92}, # 分割区域外接矩形的面积（单位：像素）占分割图像的比例最大值，用于过滤掉大面积的分割区域                              |
                        {'seg_filter_max_depth_diff': 300}, # 深度连续区域的阈值，如果某点的深度和相邻点的深度之差大于改阈值，则认为改点深度不连续，默认值100mm
                        {'seg_map_depth_count': 1}, # 分割区域至少有seg_map_depth_count个点映射到深度图上才认为是有效的分割区域
                        {'enable_pcl_cvt': False},
                        {'enable_dbg_detect': False},
                        {'enable_dbg_seg': False},
                        {'enable_pub_ai_with_depth': True},
                        {'enable_pub_map': True},
                        {'enable_tros_perf': False},
                        {'seg_valid_labels': '1'}
                ],
                arguments=['--ros-args', '--log-level', 'warn']
            ),


            # MOT node
            Node(
                package='hobot_mot',
                executable='tros_mot_node',
                parameters=[
                    {'sub_topic': LaunchConfiguration('mot_sub_topic')},
                    {'pub_topic': LaunchConfiguration('mot_pub_topic')},
                    {'mot_config_path': LaunchConfiguration('mot_config')},
                    {'frame_width': LaunchConfiguration('depth_fusion_detect_input_width')},
                    {'frame_height': LaunchConfiguration('depth_fusion_detect_input_height')}
                ],
                arguments=['--ros-args', '--log-level', log_level],
                output='screen'
            ),

            # robot control node
            Node(
                package='tros_person_following',
                executable='tros_person_following',
                parameters=[{
                    'detect_result_topic_name': LaunchConfiguration('detect_result_topic_name'),
                    'goal_pose_topic_name': LaunchConfiguration('goal_pose_topic_name'),
                    'global_frame': LaunchConfiguration('global_frame'),
                    'robot_frame': LaunchConfiguration('robot_frame'),
                    'idle_search_start_timeout_sec': LaunchConfiguration('idle_search_start_timeout_sec'),
                    'spin_radian': LaunchConfiguration('spin_radian'),
                    'explore_spin_angular_speed': LaunchConfiguration('explore_spin_angular_speed'),
                    'lost_to_idle_timeout_sec': LaunchConfiguration('lost_to_idle_timeout_sec'),
                    'tracking_to_lost_timeout_sec': LaunchConfiguration('tracking_to_lost_timeout_sec'),
                    'target_filter_width_thr': LaunchConfiguration('target_filter_width_thr'),
                    'target_filter_height_thr': LaunchConfiguration('target_filter_height_thr'),
                    'target_filter_range_x_min': LaunchConfiguration('target_filter_range_x_min'),
                    'target_filter_range_x_max': LaunchConfiguration('target_filter_range_x_max'),
                    'target_filter_range_y_min': LaunchConfiguration('target_filter_range_y_min'),
                    'target_filter_range_y_max': LaunchConfiguration('target_filter_range_y_max'),
                    'idle_search_total_timeout_sec': LaunchConfiguration('idle_search_total_timeout_sec'),
                    'idle_observe_duration_sec': LaunchConfiguration('idle_observe_duration_sec'),
                    'observe_cooldown_sec': LaunchConfiguration('observe_cooldown_sec'),
                    'follow_goal_yaw_deadzone': LaunchConfiguration('follow_goal_yaw_deadzone'),
                    'target_filter_confidence_thr': LaunchConfiguration('target_filter_confidence_thr'),
                    'select_min_confidence': LaunchConfiguration('select_min_confidence'),
                    'select_weight_dist': LaunchConfiguration('select_weight_dist'),
                    'select_weight_score': LaunchConfiguration('select_weight_score'),
                    'select_weight_center': LaunchConfiguration('select_weight_center'),
                    'follow_distance_min': LaunchConfiguration('follow_distance_min'),
                    'follow_distance_max': LaunchConfiguration('follow_distance_max'),
                    'follow_hysteresis': LaunchConfiguration('follow_hysteresis'),
                    'static_target_timeout_sec': LaunchConfiguration('static_target_timeout_sec'),
                    'static_target_move_thr': LaunchConfiguration('static_target_move_thr'),
                    'static_switch_depth_diff_thr': LaunchConfiguration('static_switch_depth_diff_thr'),
                    'static_switch_activity_window_sec': LaunchConfiguration('static_switch_activity_window_sec'),
                    'predict_window_sec': LaunchConfiguration('predict_window_sec'),
                    'predict_lead_sec': LaunchConfiguration('predict_lead_sec'),
                    'predict_max_dist': LaunchConfiguration('predict_max_dist'),
                    'predict_stale_sec': LaunchConfiguration('predict_stale_sec'),
                    'costmap_topic': LaunchConfiguration('costmap_topic'),
                    'followed_target_topic': LaunchConfiguration('followed_target_topic'),
                    'followed_target_pose_topic': LaunchConfiguration('followed_target_pose_topic'),
                    'costmap_free_cost_thr': LaunchConfiguration('costmap_free_cost_thr'),
                    'belief_search_timeout_sec': LaunchConfiguration('belief_search_timeout_sec'),
                    'belief_search_max_rounds': LaunchConfiguration('belief_search_max_rounds'),
                    'relock_dist_phase1': LaunchConfiguration('relock_dist_phase1'),
                    'relock_dist_phase2': LaunchConfiguration('relock_dist_phase2'),
                    'relock_dist_phase3': LaunchConfiguration('relock_dist_phase3'),
                    'relock_dist_phase1_sec': LaunchConfiguration('relock_dist_phase1_sec'),
                    'relock_dist_phase2_sec': LaunchConfiguration('relock_dist_phase2_sec'),
                    # Edge-of-frame turn-to-target: image_width (double, px) for
                    # ROI edge check; edge_margin_ratio is the edge band fraction.
                    'image_width': LaunchConfiguration('image_width'),
                    'edge_margin_ratio': LaunchConfiguration('edge_margin_ratio'),
                    # 蜂鸣器节流(秒)：>0 最小间隔；==0 不限制；<0 禁用蜂鸣器。
                    'buzzer_min_interval_sec': LaunchConfiguration('buzzer_min_interval_sec'),
                }],
                arguments=['--ros-args', '--log-level', log_level],
                output='screen'
            ),

            # web render node
            # websocket_image_topic / websocket_only_show_image derive from enable_perc_render:
            #   perc_render=true  → show the perception-rendered image (/tros_render_img), only_show_image=true
            #   perc_render=false → show the raw jpeg (/image_jpeg), only_show_image=false
            # Compare case-insensitively (.lower()=='true') so both 'True' (launch bool default)
            # and 'true' (user override) are accepted.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('websocket'),
                        'launch/websocket.launch.py')),
                condition=IfCondition(LaunchConfiguration('enable_websocket')),
                launch_arguments={
                    'websocket_image_topic': PythonExpression([
                        "'/tros_render_img' if '", LaunchConfiguration('enable_perc_render'),
                        "'.lower() == 'true' else '/image_jpeg'"]),
                    'websocket_only_show_image': PythonExpression([
                        "'true' if '", LaunchConfiguration('enable_perc_render'),
                        "'.lower() == 'true' else 'false'"]),
                    'websocket_smart_topic': '/tros_person_followed',
                    'websocket_channel': '1',
                    'log_level': log_level
                }.items()
            ),

            # perception render node: render the perception results on jpeg image and publish the render image
            Node(
                condition=IfCondition(LaunchConfiguration('enable_perc_render')),
                package='tros_perception_render',
                executable='tros_perception_render',
                name='tros_perception_render',
                output='screen',
                parameters=[
                    # {'perception_topic_name': '/tros_mot_targets'},
                    {'perception_topic_name': '/tros_person_followed'},
                    {'img_topic_name': '/image_jpeg'},
                    {'pub_render_topic_name': '/tros_render_img'},
                    {'render_sys_info': False},
                ],
                arguments=['--ros-args', '--log-level', 'warn']
            ),
        ],
    )

    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_respawn_cmd,
        declare_container_name_cmd,
        declare_log_level_cmd,
        declare_depth_fusion_detect_input_width_cmd,
        declare_depth_fusion_detect_input_height_cmd,
        declare_detect_result_topic_cmd,
        declare_goal_pose_topic_cmd,
        declare_global_frame_cmd,
        declare_robot_frame_cmd,
        declare_person_track_timeout_cmd,
        declare_spin_radian_cmd,
        declare_explore_spin_angular_speed_cmd,
        declare_target_find_timeout_cmd,
        declare_idle_observe_duration_sec_cmd,
        declare_observe_cooldown_sec_cmd,
        declare_timeout_sec_lost2idle_cmd,
        declare_timeout_sec_tracking2lost_cmd,
        declare_width_thr_cmd,
        declare_height_thr_cmd,
        declare_range_x_min_cmd,
        declare_range_x_max_cmd,
        declare_range_y_min_cmd,
        declare_range_y_max_cmd,
        declare_nav_goal_yaw_thr_cmd,
        declare_follow_distance_min_cmd,
        declare_follow_distance_max_cmd,
        declare_follow_hysteresis_cmd,
        declare_static_target_timeout_sec_cmd,
        declare_static_target_move_thr_cmd,
        declare_switch_depth_diff_thr_cmd,
        declare_switch_activity_window_sec_cmd,
        declare_predict_window_sec_cmd,
        declare_predict_lead_sec_cmd,
        declare_predict_max_dist_cmd,
        declare_predict_stale_sec_cmd,
        declare_costmap_topic_cmd,
        declare_followed_target_topic_cmd,
        declare_followed_target_pose_topic_cmd,
        declare_goal_free_cost_thr_cmd,
        declare_belief_search_timeout_sec_cmd,
        declare_belief_search_max_rounds_cmd,
        declare_relock_dist_phase1_cmd,
        declare_relock_dist_phase2_cmd,
        declare_relock_dist_phase3_cmd,
        declare_relock_dist_phase1_sec_cmd,
        declare_relock_dist_phase2_sec_cmd,
        declare_confidence_thr_cmd,
        declare_select_min_confidence_cmd,
        declare_select_weight_dist_cmd,
        declare_select_weight_score_cmd,
        declare_select_weight_center_cmd,
        declare_edge_margin_ratio_cmd,
        declare_image_width_cmd,
        declare_mot_sub_topic_cmd,
        declare_mot_pub_topic_cmd,
        declare_mot_config_cmd,
        declare_enable_perc_render_cmd,
        declare_enable_websocket_cmd,
        declare_buzzer_min_interval_sec_cmd,
        load_nodes,
    ])
