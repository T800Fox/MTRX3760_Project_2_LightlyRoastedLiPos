cmake_minimum_required(VERSION 3.8)
project(mtrx3760_oogway_mazesolver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#---Find Dependencies---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#---Include .hpp's---
include_directories(include)

install(DIRECTORY include/${PROJECT_NAME}/
          DESTINATION include/${PROJECT_NAME}
          FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")

# --- Install launch & world files ---
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY worlds
        DESTINATION share/${PROJECT_NAME})

#---Custom Message Setup---
set(msg_files
  "msg/WallPair.msg"
  "msg/ActuatorCommand.msg"
  "msg/NavCommand.msg"    
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs nav_msgs
)

#---Export Dependencies---
ament_export_dependencies(
  rclcpp 
  std_msgs sensor_msgs geometry_msgs nav_msgs
  rosidl_default_runtime)

#---Individual Nodes--- 
add_executable(mazeNavigator src/mazeNavigator_node.cpp src/leftWallFollower.cpp)
ament_target_dependencies(mazeNavigator rclcpp std_msgs geometry_msgs nav_msgs)

add_executable(wallLocator src/wallLocator_node.cpp)
ament_target_dependencies(wallLocator rclcpp sensor_msgs)

add_executable(goalChecker src/goalChecker_node.cpp)
ament_target_dependencies(goalChecker rclcpp nav_msgs std_msgs)

add_executable(cameraViewer src/cameraViewer_node.cpp)
ament_target_dependencies(cameraViewer rclcpp sensor_msgs)

#---Build Testing?---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#---Install Targets---
install(TARGETS
  mazeNavigator
  wallLocator
  goalChecker
  cameraViewer
  DESTINATION lib/${PROJECT_NAME}
)

#---Link Messages Against Interface of Each Node---
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp
)
target_link_libraries(wallLocator "${cpp_typesupport_target}")
target_link_libraries(mazeNavigator "${cpp_typesupport_target}")

ament_package()
