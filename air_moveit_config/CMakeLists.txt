cmake_minimum_required(VERSION 3.1.3)
project(air_moveit_config)

find_package(catkin REQUIRED COMPONENTS
	rospy
	roscpp
	moveit_core
	moveit_task_constructor_core
	moveit_ros_planning_interface
	rosparam_shortcuts
	pcl_conversions
	pcl_ros
	moveit_ros_perception
	)
find_package(Eigen3 REQUIRED)
moveit_build_options()

include_directories(
 ${catkin_INCLUDE_DIRS}
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp
	DEPENDS EIGEN3
)

add_library(${PROJECT_NAME}_pick_place_task src/pick_place_task.cpp)
target_link_libraries(${PROJECT_NAME}_pick_place_task ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME}_pick_place_task PUBLIC include)
target_include_directories(${PROJECT_NAME}_pick_place_task SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
add_dependencies(${PROJECT_NAME}_pick_place_task ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS ${PROJECT_NAME}_pick_place_task
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# declare a demo consisting of a single cpp file
function(demo name)
	add_executable(${PROJECT_NAME}_${name} src/${name}.cpp)
	add_dependencies(${PROJECT_NAME}_${name} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
	target_link_libraries(${PROJECT_NAME}_${name} ${catkin_LIBRARIES})
	target_include_directories(${PROJECT_NAME}_${name} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
	set_target_properties(${PROJECT_NAME}_${name} PROPERTIES OUTPUT_NAME ${name} PREFIX "")
	install(TARGETS ${PROJECT_NAME}_${name}
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
endfunction()

demo(cartesian)
demo(modular)
demo(alternative_path_costs)
demo(ik_clearance_cost)
demo(fallbacks_move_to)
demo(detect_and_add_cylinder_collision_object_demo)
demo(pick_place_demo)
demo(detect_and_add)
demo(add_sphere)
demo(add_scene)
demo(sub_col)

target_link_libraries(${PROJECT_NAME}_pick_place_demo ${PROJECT_NAME}_pick_place_task)

install(DIRECTORY launch config
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_subdirectory(test)
