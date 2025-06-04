cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_interfaces REQUIRED)

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

# Define minimal_publisher target and link libraries
add_executable(minimal_publisher src/minimal_publisher.cpp)
ament_target_dependencies(minimal_publisher rclcpp example_interfaces)

# Define minimal_subscriber target and link libraries
add_executable(minimal_subscriber src/minimal_subscriber.cpp)
ament_target_dependencies(minimal_subscriber rclcpp example_interfaces)

# Define minimal_server target and link libraries
add_executable(minimal_server src/minimal_server.cpp)
ament_target_dependencies(minimal_server rclcpp example_interfaces)

# Define minimal_client target and link libraries
add_executable(minimal_client src/minimal_client.cpp)
ament_target_dependencies(minimal_client rclcpp example_interfaces)

# Define acc_subscriber target and link libraries
add_executable(acc_subscriber src/acc_subscriber.cpp)
ament_target_dependencies(acc_subscriber rclcpp my_interfaces)

# Define trigger_points_client target and link libraries
add_executable(trigger_points_client src/trigger_points_client.cpp)
ament_target_dependencies(trigger_points_client rclcpp my_interfaces)

# Define publisher_with_params target and link libraries
add_executable(publisher_with_params src/publisher_with_params.cpp)
ament_target_dependencies(publisher_with_params rclcpp example_interfaces)

# Install binaries to specific folder
install(TARGETS 
  minimal_publisher
  minimal_subscriber
  minimal_server
  minimal_client
  acc_subscriber
  trigger_points_client
  publisher_with_params
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
