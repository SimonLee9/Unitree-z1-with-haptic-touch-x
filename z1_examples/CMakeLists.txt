cmake_minimum_required(VERSION 3.1.3)
project(z1_examples)

find_package(catkin REQUIRED COMPONENTS
    moveit_ros_planning_interface
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  LIBRARIES
    moveit_motion_planning_rviz_plugin_core
    moveit_planning_scene_rviz_plugin_core

  #INCLUDE_DIRS
  #  motion_planning_rviz_plugin/include # for getting Torque data
  #  planning_scene_rviz_plugin/include  # for getting Torque data
  
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)

include_directories(rviz_plugin_render_tools/include
                    robot_state_rviz_plugin/include
                    planning_scene_rviz_plugin/include
                    motion_planning_rviz_plugin/include
                    trajectory_rviz_plugin/include)

                   

include_directories(SYSTEM
 ${Boost_INCLUDE_DIRS}
 ${z1_sdk_INCLUDE_DIRS}
 ${catkin_INCLUDE_DIRS}
 include
)

# examples
set(EXAMPLE_FILES
  examples/ros_gripper_ctrl.cpp
  examples/move_group_interface.cpp
  examples/move_group_interface_home.cpp
  examples/move_group_interface_tcp.cpp
  examples/torque.cpp
)

foreach(EXAMPLE_FILE IN LISTS EXAMPLE_FILES)
  get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)
  add_executable(${EXAMPLE_NAME} ${EXAMPLE_FILE})
  target_link_libraries(${EXAMPLE_NAME} ${catkin_LIBRARIES} )
endforeach()

#add_subdirectory(planning_scene_rviz_plugin) # for getting Torque data
#add_subdirectory(motion_planning_rviz_plugin) # for getting Torque data

install(FILES
  motion_planning_rviz_plugin_description.xml
  planning_scene_rviz_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})