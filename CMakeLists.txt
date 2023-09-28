cmake_minimum_required(VERSION 3.5)
project(centauri_6dof)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(urdf REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)


include_directories(${Eigen_INCLUDE_DIRS})

# find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
)
set(REF_GEN_INCLUDE_DEPENDS
  kdl_parser
  rclcpp
  trajectory_msgs
)
set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  realtime_tools
  trajectory_msgs
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# # Create the executable for your C++ node
# add_executable(joint_states_subscriber_node src/joint_states_subscriber_node.cpp)

# # Link the executable with ROS 2 libraries
# target_link_libraries(joint_states_subscriber_node
#   rclcpp
#   sensor_msgs
# )

# # Install the executable
# install(TARGETS
# joint_states_subscriber_node
#   DESTINATION lib/${PROJECT_NAME}
# )

add_executable(send_target_to_moveit_node src/send_target_to_moveit.cpp)
# Especificar las dependencias del ejecutable
ament_target_dependencies(send_target_to_moveit_node
ament_cmake
Boost
control_msgs
Eigen3
interactive_markers
moveit_core
moveit_msgs
moveit_ros_planning
moveit_ros_planning_interface
moveit_servo
moveit_visual_tools
pluginlib
rclcpp
rclcpp_action
rviz_visual_tools
tf2_geometry_msgs
tf2_geometry_msgs
tf2_ros
)

# Instalar el ejecutable
install(TARGETS send_target_to_moveit_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(object_frame_listener src/object_frame_listener.cpp)
ament_target_dependencies(object_frame_listener
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
  std_msgs
)

install(TARGETS
object_frame_listener
  DESTINATION lib/${PROJECT_NAME}
)


# Mark the package for export
ament_export_dependencies(
  ament_cmake
  Boost
  control_msgs
  Eigen3
  interactive_markers
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_servo
  moveit_visual_tools
  pluginlib
  rclcpp
  rclcpp_action
  rviz_visual_tools
  tf2_geometry_msgs
  tf2_geometry_msgs
  tf2_ros
  tf2
  geometry_msgs


)

# Install URDF files
install(DIRECTORY config launch meshes models rviz src test worlds ros2_control xacro
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
