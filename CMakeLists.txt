################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(myahrs_driver)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  tf
)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES myahrs_driver
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs tf
  DEPENDS
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(myahrs_driver src/myahrs_driver.cpp)
add_dependencies(myahrs_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(myahrs_driver ${catkin_LIBRARIES})

# I knew the REP0003(http://www.ros.org/reps/rep-0003.html#c),
# but the code of "myAHRS SDK" must compile with C++11.
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


################################################################################
# Install
################################################################################
install(TARGETS myahrs_driver
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY rviz_cfg
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
#############################################################################
