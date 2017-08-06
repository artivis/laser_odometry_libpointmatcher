cmake_minimum_required(VERSION 2.8.3)
project(laser_odometry_libpointmatcher)

# Set compiler according C++11 support
# Default for the CXX_STANDARD target property
set(CMAKE_CXX_STANDARD 11)
# Prevent fallback behaviour to anterior version
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Avoid linking to a different library
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
    laser_odometry_core roscpp tf_conversions #rosparam_handler dynamic_reconfigure
)

find_package(libpointmatcher COMPONENTS pointmatcher REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS laser_odometry_core roscpp tf_conversions #rosparam_handler dynamic_reconfigure
  DEPENDS libpointmatcher
)

include_directories(include ${catkin_INCLUDE_DIRS} ${libpointmatcher_INCLUDE_DIRS})

###############
##   Build   ##
###############

add_library(${PROJECT_NAME} src/laser_odometry_libpointmatcher.cpp src/conversion.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${libpointmatcher_INCLUDE_DIRS})

################
##  Install   ##
################

install(TARGETS ${PROJECT_NAME}
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES laser_odometry_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

################
##    Test    ##
################

#if(CATKIN_ENABLE_TESTING)
#    find_package(rostest REQUIRED)
#    find_package(catkin REQUIRED COMPONENTS
#        roslib
#    )

#    add_rostest_gtest(test_carmen_reader_test
#      test/test_carmen_reader.test
#      test/test_carmen_reader.cpp
#    )

#  target_link_libraries(test_carmen_reader_test ${catkin_LIBRARIES} ${PROJECT_NAME})

#endif()
