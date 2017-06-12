add_library(modify_driver modify_driver.cpp)
target_link_libraries(modify_driver
                      ${catkin_LIBRARIES})

 add_executable(fullscan_puber fullscan_puber.cpp)
 #add_dependencies(talker ${${PROJECT_NAME}_EXPORTED_TARGETS})
 target_link_libraries(fullscan_puber 
	velodyne_input
	${catkin_LIBRARIES}
	${libpcap_LIBRARIES}
        modify_driver
)
