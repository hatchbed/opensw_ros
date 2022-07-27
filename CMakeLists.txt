cmake_minimum_required(VERSION 3.1)
project(rpad_ros VERSION 0.1.0)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

find_package(PkgConfig)
pkg_check_modules(rpad REQUIRED rpad)
link_directories(${rpad_LIBRARY_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${rpad_INCLUDE_DIRS}
)

catkin_package()

add_executable(laser_driver src/laser_driver.cpp src/logger.cpp)
target_link_libraries(laser_driver ${catkin_LIBRARIES} ${rpad_LIBRARIES})
set_property(TARGET laser_driver PROPERTY CXX_STANDARD 17)

install(TARGETS laser_driver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
