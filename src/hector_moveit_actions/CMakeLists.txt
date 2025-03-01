cmake_minimum_required(VERSION 2.8.3)
set (CMAKE_CXX_STANDARD 11)
project(hector_moveit_actions)

find_package(catkin REQUIRED COMPONENTS 
	moveit_core 
	roscpp 
	actionlib 
	actionlib_msgs 
	geometry_msgs 
	hector_uav_msgs 
	trajectory_msgs 
	message_generation
	control_msgs)
	
find_package(Boost REQUIRED COMPONENTS system)
add_action_files(
  DIRECTORY action
  FILES ExecuteDroneTrajectory.action FollowMultiDofJointTrajectory.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  trajectory_msgs
  control_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp actionlib_msgs hector_uav_msgs trajectory_msgs
)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(trajectory_controller src/trajectory_executer.cpp)
add_executable(follow_multi_dof_joint_trajectory_action_server src/follow_multi_dof_joint_trajectory_action_server.cpp)

target_link_libraries(trajectory_controller ${catkin_LIBRARIES})
target_link_libraries(follow_multi_dof_joint_trajectory_action_server ${catkin_LIBRARIES})

add_dependencies(
  trajectory_controller
  ${hector_moveit_actions_EXPORTED_TARGETS} hector_uav_msgs_generate_messages
)

add_dependencies(
  follow_multi_dof_joint_trajectory_action_server
  ${hector_moveit_actions_EXPORTED_TARGETS} hector_uav_msgs_generate_messages 
)
