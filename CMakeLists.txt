cmake_minimum_required(VERSION 3.8)
project(tros_person_following)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -ggdb)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(angles REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

add_library(${PROJECT_NAME}_component SHARED
  src/person_following_node.cpp
)

target_compile_features(${PROJECT_NAME}_component PUBLIC cxx_std_17)

include_directories(include)

ament_target_dependencies(${PROJECT_NAME}_component
  "rclcpp"
  "rclcpp_components"
  "rclcpp_action"
  "nav2_msgs"
  "nav2_util"
  "geometry_msgs"
  "nav_msgs"
  "tf2_ros"
  "angles"
  "ai_msgs"
  "std_srvs"
  "std_msgs"
  "builtin_interfaces"
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "tros_person_following::PersonFollowingNode"
  EXECUTABLE ${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_component
  DESTINATION lib/)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# MOT config json (copied from hobot_mot so this package owns the MOT params
# it runs with; launch passes its absolute path to tros_mot_node's mot_config_path).
install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
