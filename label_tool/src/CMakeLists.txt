add_library(track_label_xcon track_label.cpp)
target_link_libraries(track_label_xcon
                      ${catkin_LIBRARIES})

#add_library(ShapeEstimator_label ShapeEstimator.cpp)
#target_link_libraries(ShapeEstimator_label
#                ${catkin_LIBRARIES})

#add_library(featureExtract featureExtract.cpp)
#target_link_libraries(featureExtract
#                    ${catkin_LIBRARIES} ShapeEstimator_label)

add_executable(example example.cpp)
add_dependencies(example ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(example ${catkin_LIBRARIES} track_label_xcon)

#add_executable(featureExtract_node featureExtract_node.cpp)
#add_dependencies(featureExtract_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
#target_link_libraries(featureExtract_node ${catkin_LIBRARIES} track_label_xcon featureExtract ShapeEstimator_label)


