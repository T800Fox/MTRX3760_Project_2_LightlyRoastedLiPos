cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0009 NEW)
project(cv_stuffs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#---Find Dependencies---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

#CAMERA
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})


#---Include .hpp's---
include_directories(include)

install(DIRECTORY include/${PROJECT_NAME}/
          DESTINATION include/${PROJECT_NAME}
          FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")



#---Custom Message Setup---
set(msg_files
  "msg/WallDist.msg"
  "msg/LinearCmd.msg"
  "msg/AngularCmd.msg"
  "msg/Pose.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)


#---Export Dependancies---
ament_export_dependencies(
  rclcpp 
  std_msgs sensor_msgs
  rosidl_default_runtime)



#---Individual Nodes--- 

add_executable(camera src/camera_node.cpp)
ament_target_dependencies(camera rclcpp std_msgs sensor_msgs cv_bridge)
target_link_libraries(camera ${OpenCV_LIBRARIES})


#---Build Testing?---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


#---Install Targets---
install(TARGETS
  camera
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

#---Link Mesages Against Interface of Each Node---
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp
)

target_link_libraries(camera "${cpp_typesupport_target}")

ament_package()
