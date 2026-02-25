cmake_minimum_required(VERSION 3.8)
project(lio_nav_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
# nav2_bringup: optional — comment out if not installed
# find_package(nav2_bringup REQUIRED)
find_package(fast_lio REQUIRED)

# PointCloud Merger Node
add_executable(pointcloud_merger src/pointcloud_merger.cpp)
target_include_directories(pointcloud_merger PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pointcloud_merger ${PCL_LIBRARIES})
ament_target_dependencies(pointcloud_merger
  rclcpp
  sensor_msgs
  message_filters
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
)

install(TARGETS pointcloud_merger
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/livox_republisher.py
  scripts/path_planner_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
