cmake_minimum_required(VERSION 2.8.3)
project(iiwa_stack_motion_test)
SET(CMAKE_CXX_FLAGS "-std=c++11")

option( ENABLE_FRI "Enable the FRI client" ON )

if( ENABLE_FRI )
  set( CMAKE_POSITION_INDEPENDENT_CODE ON)
  add_definitions( -DENABLE_FRI )
  SET( KUKA_FRI_LIBRARIES FRIClient )
  SET( KUKA_FRI_INCLUDE  /usr/local/include )
endif()


set(MSG_DEPS
    std_msgs)
    

find_package(catkin REQUIRED 
  COMPONENTS
  roscpp
  tf
  tf2
  geometry_msgs
  sensor_msgs
  iiwa_ros
  std_msgs
  message_filters
  ${MSG_DEPS}
)

find_package( Boost REQUIRED
		    COMPONENTS
		    signals
		    )


catkin_package(	
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} 
  CATKIN_DEPENDS std_msgs
		 roscpp
		 tf
		 tf2
		 geometry_msgs
		 sensor_msgs
		 std_msgs
		 message_filters
		 
  DEPENDS system_lib Boost
)

include_directories(include 
		     ${catkin_INCLUDE_DIRS}
		     ${Boost_INCLUDE_DIRS})
include_directories( ${KUKA_FRI_INCLUDE}   )


add_executable(iiwa_stack_motion_test_node                     src/vertical_motion_test.cpp)
add_executable(iiwa_variable_impedance_data_logger_node        src/iiwa_variable_impedance_data_logger.cpp)
add_executable(iiwa_initial_random_impedance_commander_node    src/iiwa_initial_random_impedance_commander.cpp)
add_executable(iiwa_MBRL_interface_node                        src/iiwa_MBRL_interface.cpp)



add_dependencies(iiwa_stack_motion_test_node  iiwa_msgs_generate_messages_cpp  ${catkin_EXPORTED_TARGETS})
add_dependencies(iiwa_variable_impedance_data_logger_node                      ${catkin_EXPORTED_TARGETS} ${Boost_EXPORTERD_TARGETS})
add_dependencies(iiwa_initial_random_impedance_commander_node                  ${catkin_EXPORTED_TARGETS})
add_dependencies(iiwa_MBRL_interface_node                                      ${catkin_EXPORTED_TARGETS})



target_link_libraries(iiwa_stack_motion_test_node                         ${catkin_LIBRARIES} ${KUKA_FRI_LIBRARIES})
target_link_libraries(iiwa_variable_impedance_data_logger_node            ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${KUKA_FRI_LIBRARIES})
target_link_libraries(iiwa_initial_random_impedance_commander_node        ${catkin_LIBRARIES} ${KUKA_FRI_LIBRARIES})
target_link_libraries(iiwa_MBRL_interface_node                            ${catkin_LIBRARIES} ${KUKA_FRI_LIBRARIES})


