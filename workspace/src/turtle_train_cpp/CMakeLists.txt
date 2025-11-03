cmake_minimum_required(VERSION 3.8)
project(turtle_train_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Include directories
include_directories(include)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Spawner executable
add_executable(spawner src/spawner.cpp)
ament_target_dependencies(
  spawner
  rclcpp
  turtlesim
  std_srvs
)

# Pose broadcaster executable
add_executable(pose_broadcaster src/pose_broadcaster.cpp)
ament_target_dependencies(
  pose_broadcaster
  rclcpp
  turtlesim
  geometry_msgs
  tf2
  tf2_ros
  std_srvs
)

# Follower executable
add_executable(follower src/follower.cpp)
ament_target_dependencies(
  follower
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_srvs
)

# Install executables
install(TARGETS
  spawner
  pose_broadcaster
  follower
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
