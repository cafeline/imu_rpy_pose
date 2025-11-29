cmake_minimum_required(VERSION 3.8)
project(imu_rpy_pose)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(imu_processor src/imu_processor.cpp)
target_include_directories(imu_processor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(imu_processor PUBLIC cxx_std_17)
ament_target_dependencies(imu_processor rclcpp)
target_link_libraries(imu_processor Eigen3::Eigen)

add_executable(imu_rpy_pose_node src/imu_rpy_pose_node.cpp)
target_include_directories(imu_rpy_pose_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(imu_rpy_pose_node PUBLIC cxx_std_17)
target_link_libraries(imu_rpy_pose_node imu_processor)
ament_target_dependencies(imu_rpy_pose_node
  rclcpp
  sensor_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS imu_rpy_pose_node imu_processor
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_imu_processor test/test_imu_processor.cpp)
  target_link_libraries(test_imu_processor imu_processor)
  ament_target_dependencies(test_imu_processor rclcpp)

  ament_add_gtest(test_yaw_publisher
    test/test_yaw_publisher.cpp)
  target_link_libraries(test_yaw_publisher imu_processor)
  ament_target_dependencies(test_yaw_publisher
    rclcpp
    sensor_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
  )
endif()

ament_package()
