cmake_minimum_required(VERSION 3.8)
project(drones_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 轨迹文件路径通过参数 trajectory_file 传入（见 launch / params）
add_executable(trajectory_follower src/trajectory_follower.cpp)
target_include_directories(trajectory_follower PRIVATE include)
ament_target_dependencies(trajectory_follower rclcpp px4_msgs std_msgs)

add_executable(trajectory_visualizer src/trajectory_visualizer.cpp)
target_include_directories(trajectory_visualizer PRIVATE include)
ament_target_dependencies(trajectory_visualizer rclcpp px4_msgs nav_msgs visualization_msgs)

add_executable(obstacle_visualizer src/obstacle_visualizer.cpp)
target_include_directories(obstacle_visualizer PRIVATE include)
ament_target_dependencies(obstacle_visualizer rclcpp visualization_msgs)

install(TARGETS trajectory_follower trajectory_visualizer obstacle_visualizer
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

install(DIRECTORY src DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.csv")
install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
