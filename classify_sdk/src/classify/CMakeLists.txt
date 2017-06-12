file(GLOB libSo ${PROJECT_SOURCE_DIR}/libso/*.so)

add_executable(classify_node classify_node.cpp)
add_dependencies(classify_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(classify_node  ${catkin_LIBRARIES} ${libSo})
