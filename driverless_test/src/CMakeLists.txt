add_library(serial_open serial_open.cpp)
target_link_libraries(serial_open 
                      ${catkin_LIBRARIES})
                      
add_library(gps_gps_com gps_com.cpp)
target_link_libraries(gps_gps_com
					  serial_open
                      ${catkin_LIBRARIES})
                 
add_library(upper_control upper_control.cpp)
target_link_libraries(upper_control
                      ${catkin_LIBRARIES})
                           
add_library(lower_com lower_com.cpp)
target_link_libraries(lower_com
					  serial_open
                      ${catkin_LIBRARIES})
                      
add_executable(serial_open serial_open.cpp)

if(catkin_EXPORTED_TARGETS)
  add_dependencies(gps_gps_com ${catkin_EXPORTED_TARGETS})
  add_dependencies(upper_control ${catkin_EXPORTED_TARGETS})
  add_dependencies(lower_com ${catkin_EXPORTED_TARGETS})
endif()

#add_dependencies(talker ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(serial_open
	gps_gps_com
	${catkin_LIBRARIES}
)

target_link_libraries(serial_open
	upper_control
	${catkin_LIBRARIES}
)

target_link_libraries(serial_open
	lower_com
	${catkin_LIBRARIES}
)
 #install(TARGETS my_signal
  #       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
