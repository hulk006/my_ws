file(GLOB libSo ${PROJECT_SOURCE_DIR}/libso/*.so)

add_executable(lidarClusterNode lidarClusterNode.cpp)
add_dependencies(lidarClusterNode ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(lidarClusterNode  ${catkin_LIBRARIES} ${libSo})

add_executable(lidarTerrianNode lidarTerrianNode.cpp)
add_dependencies(lidarTerrianNode ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(lidarTerrianNode  ${catkin_LIBRARIES} ${libSo})
