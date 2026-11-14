cmake_minimum_required(VERSION 3.8)
project(fastbot_waypoints)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message Generation
ament_export_dependencies(rosidl_default_runtime)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/WaypointAction.action"
  DEPENDENCIES 
  geometry_msgs
)

# Library
include_directories(include)
add_library(fastbot_action_server_lib src/fastbot_action_server.cpp include/fastbot_waypoints/fastbot_action_server.hpp)
ament_target_dependencies(fastbot_action_server_lib 
  rclcpp
  rclcpp_action
  tf2
  geometry_msgs
  nav_msgs
  action_msgs
  tf2_geometry_msgs
  rosidl_default_runtime)

ament_export_targets(fastbot_action_server_lib HAS_LIBRARY_TARGET)

install(
	DIRECTORY include/fastbot_waypoints
	DESTINATION include
)


# Executable
add_executable(fastbot_action_server src/main.cpp)
target_link_libraries(
	fastbot_action_server fastbot_action_server_lib
)

if(COMMAND rosidl_get_typesupport_target)
  rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(fastbot_action_server_lib "${typesupport_target}")
else()
  rosidl_target_interfaces(fastbot_action_server_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()


# Install
install(TARGETS fastbot_action_server_lib
	DESTINATION lib/fastbot_action_server_lib)

install(TARGETS 
	fastbot_action_server
	DESTINATION lib/${PROJECT_NAME}/)

install(
	TARGETS fastbot_action_server_lib
	EXPORT fastbot_action_server_lib
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)


# Testing
if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	find_package(ament_cmake_gtest REQUIRED)

	set(TESTFILES
		test/main.cpp
		test/waypoint_action_test.cpp
		)
	# Add a gtest executables
	ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
	target_link_libraries(${PROJECT_NAME}_test fastbot_action_server_lib)

	install(TARGETS
		${PROJECT_NAME}_test
		# Install the generated test executable file to the path after DESTINATION
		DESTINATION lib/${PROJECT_NAME})
	# uncomment to enforce lintners and static code analyzers defined in ament_lint_common package        
	# ament_lint_auto_find_test_dependencies()
endif()

ament_package()
