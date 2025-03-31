cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Interface generation (actions + services)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoToPose.action"
  "srv/GetDirection.srv"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

# Link directories
link_directories(/opt/homebrew/opt/box2d/lib /opt/homebrew/opt/glfw/lib)

# Nodes
add_executable(patrol_node src/patrol.cpp)
ament_target_dependencies(patrol_node rclcpp std_msgs geometry_msgs sensor_msgs)

add_executable(direction_service_node src/direction_service.cpp)
ament_target_dependencies(direction_service_node rclcpp std_msgs sensor_msgs)

add_executable(test_service_node src/test_service.cpp)
ament_target_dependencies(test_service_node rclcpp sensor_msgs)

add_executable(patrol_with_service_node src/patrol_with_service.cpp)
ament_target_dependencies(patrol_with_service_node rclcpp std_msgs geometry_msgs sensor_msgs)

add_executable(go_to_pose_action_node src/go_to_pose_action.cpp)
ament_target_dependencies(go_to_pose_action_node rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs rclcpp_action action_msgs)

# Link generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(go_to_pose_action_node ${cpp_typesupport_target})
target_link_libraries(direction_service_node ${cpp_typesupport_target})
target_link_libraries(test_service_node ${cpp_typesupport_target})
target_link_libraries(patrol_with_service_node ${cpp_typesupport_target})
target_link_libraries(patrol_node ${cpp_typesupport_target})

# Install targets
install(TARGETS
  patrol_node
  direction_service_node
  test_service_node
  patrol_with_service_node
  go_to_pose_action_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install interface and config folders
install(DIRECTORY
  launch
  config
  action
  srv
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
