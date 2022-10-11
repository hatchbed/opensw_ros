cmake_minimum_required(VERSION 3.5)
project(rpad_ros VERSION 0.1.0)

find_package(ament_cmake REQUIRED)
find_package(hatchbed_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(PkgConfig)
pkg_check_modules(rpad REQUIRED rpad)
link_directories(${rpad_LIBRARY_DIRS})

include_directories(
  include
  ${rpad_INCLUDE_DIRS}
)

add_executable(imu_driver src/imu_driver.cpp src/logger.cpp)
target_link_libraries(imu_driver ${rpad_LIBRARIES})
set_property(TARGET imu_driver PROPERTY CXX_STANDARD 17)
ament_target_dependencies(imu_driver hatchbed_common rclcpp sensor_msgs)

add_executable(laser_driver src/laser_driver.cpp src/logger.cpp)
target_link_libraries(laser_driver ${rpad_LIBRARIES})
set_property(TARGET laser_driver PROPERTY CXX_STANDARD 17)
ament_target_dependencies(laser_driver hatchbed_common rclcpp sensor_msgs)

install(
  TARGETS 
    imu_driver
    laser_driver
  DESTINATION lib/${PROJECT_NAME})

ament_package()
