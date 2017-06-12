add_library(classifyWithXgboost classifyWithXgboost.cpp)
target_link_libraries(classifyWithXgboost
        ${catkin_LIBRARIES})

add_library(xgbTrack xgbclassifyTrack.cpp)
target_link_libraries(xgbTrack
        ${catkin_LIBRARIES} track drawing classifyWithXgboost)

add_executable(xgb_node xgb_classify_node.cpp)
add_dependencies(xgb_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(xgb_node  ${catkin_LIBRARIES} xgbTrack)



