cmake_minimum_required(VERSION 2.8.3)
project(lidar_lane_detector)

find_package(PCL REQUIRED)

find_package(Eigen3 QUIET)

if (NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        roscpp
        std_msgs
        nav_msgs
        tf
        pcl_ros
        sensor_msgs
        autoware_msgs
        pcl_conversions
        velodyne_pointcloud
        )

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS std_msgs velodyne_pointcloud autoware_msgs
        DEPENDS PCL LIBRARIES libpolynomial
)

###########
## Build ##
###########
add_library(libpolynomial lib/polynomial.cpp)
add_dependencies(libpolynomial ${catkin_EXPORTED_TARGETS})

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_library(points_accumulator nodes/points_accumulator/points_accumulator.cpp)
target_link_libraries(points_accumulator ${catkin_LIBRARIES})
add_dependencies(points_accumulator ${catkin_EXPORTED_TARGETS})

add_executable(points_accumulator_node nodes/points_accumulator/points_accumulator_node.cpp)
target_link_libraries(points_accumulator_node ${catkin_LIBRARIES} points_accumulator)
add_dependencies(points_accumulator_node ${catkin_EXPORTED_TARGETS})

add_library(lane_detector nodes/lane_detector/lane_detector.cpp)
target_link_libraries(lane_detector ${catkin_LIBRARIES} libpolynomial)
add_dependencies(lane_detector ${catkin_EXPORTED_TARGETS})

add_executable(lane_detector_node nodes/lane_detector/lane_detector_node.cpp)
target_link_libraries(lane_detector_node ${catkin_LIBRARIES} lane_detector)
add_dependencies(lane_detector_node ${catkin_EXPORTED_TARGETS})

install(TARGETS points_accumulator_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(TARGETS lane_detector_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
