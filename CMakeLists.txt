cmake_minimum_required(VERSION 3.8)
project(drones_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

# 把包内 src 目录的绝对路径编译进去，方便节点默认定位 uav0.csv
add_executable(trajectory_follower src/trajectory_follower.cpp)
target_include_directories(trajectory_follower PRIVATE include)
target_compile_definitions(trajectory_follower PRIVATE
  "PKG_SRC_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/src\"")
ament_target_dependencies(trajectory_follower rclcpp px4_msgs)

install(TARGETS trajectory_follower
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
