add_library(classifier classifier.cpp)
target_link_libraries(classifier
                      ${catkin_LIBRARIES})

add_library(boxEstimater boxEstimater.cpp)
target_link_libraries(boxEstimater
                  ${catkin_LIBRARIES})

add_library(track track.cpp)
target_link_libraries(track
                    ${catkin_LIBRARIES} boxEstimater)

add_library(drawing drawing.cpp)
target_link_libraries(drawing 
                    ${catkin_LIBRARIES} track)


add_library(ShapeEstimator ShapeEstimator.cpp)
target_link_libraries(ShapeEstimator
                    ${catkin_LIBRARIES})

add_library(FeatureManager FeatureManager.cpp)
target_link_libraries(FeatureManager
                    ${catkin_LIBRARIES} ShapeEstimator boxEstimater)

add_library(classifySVM classifySVM.cpp)
target_link_libraries(classifySVM
                      ${catkin_LIBRARIES} classifier FeatureManager)

add_library(classifyTrack classifyTrack.cpp)
target_link_libraries(classifyTrack
                    ${catkin_LIBRARIES} classifySVM track drawing)

add_library(trackManager trackManager.cpp)
target_link_libraries(trackManager
                    ${catkin_LIBRARIES})

add_executable(classify_track_node svm_classify_node.cpp)
add_dependencies(classify_track_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(classify_track_node  ${catkin_LIBRARIES} classifyTrack)

add_executable(trainFileGenerate trainFileGenerate.cpp)
add_dependencies(trainFileGenerate ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(trainFileGenerate  ${catkin_LIBRARIES} trackManager FeatureManager)
