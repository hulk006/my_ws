cmake_minimum_required(VERSION 2.8.3)
project(lidar_track)

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        genmsg
        pcl_conversions
        pcl_ros
        roscpp
        rospy
        std_msgs
        )
find_package(Boost REQUIRED thread signals)
#find_package(Eigen REQUIRED)
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
find_package(PCL REQUIRED COMPONENTS common kdtree io)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES Robosense_cluster
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

add_definitions(${EIGEN_DEFINITIONS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_library (precision_tracking
        src/precision_tracking/adh_tracker3d.cpp
        src/precision_tracking/alignment_evaluator.cpp
        src/precision_tracking/density_grid_2d_evaluator.cpp
        src/precision_tracking/density_grid_3d_evaluator.cpp
        src/precision_tracking/down_sampler.cpp
        src/precision_tracking/high_res_timer.cpp
        src/precision_tracking/motion_model.cpp
        src/precision_tracking/precision_tracker.cpp
        src/precision_tracking/scored_transform.cpp
        src/precision_tracking/sensor_specs.cpp
        src/precision_tracking/track_manager.cpp
        src/precision_tracking/tracker.cpp

        include/precision_tracking/params.h
        include/precision_tracking/adh_tracker3d.h
        include/precision_tracking/alignment_evaluator.h
        include/precision_tracking/density_grid_2d_evaluator.h
        include/precision_tracking/density_grid_3d_evaluator.h
        include/precision_tracking/down_sampler.h
        include/precision_tracking/high_res_timer.h
        include/precision_tracking/motion_model.h
        include/precision_tracking/precision_tracker.h
        include/precision_tracking/scored_transform.h
        include/precision_tracking/track_manager.h
        include/precision_tracking/sensor_specs.h
        include/precision_tracking/tracker.h
        )
target_link_libraries(precision_tracking ${EIGEN_LIBRARIES} ${PCL_LIBRARIES})

###########
## Build ##
###########

add_library(Tracking src/Tracking.cpp include/Tracking.h)
target_link_libraries(Tracking
        ${catkin_LIBRARIES} precision_tracking drawRviz)

add_library(drawRviz src/drawRviz.cpp include/drawRviz.h)
target_link_libraries(drawRviz ${catkin_LIBRARIES})

add_library(Clustering src/roboCluster.cpp include/roboCluster.h)
target_link_libraries(Clustering ${catkin_LIBRARIES})

add_executable(lidarTrackNode nodes/lidarTrackNode.cpp)
target_link_libraries(lidarTrackNode ${catkin_LIBRARIES} Tracking Clustering)


