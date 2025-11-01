cmake_minimum_required(VERSION 3.8)
project(mtrx3760_lrl_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)


#---Include .hpp's---
include_directories(include)

install(DIRECTORY include/${PROJECT_NAME}/
          DESTINATION include/${PROJECT_NAME}
          FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")



ament_export_dependencies(
  rclcpp rclcpp_action
  std_msgs sensor_msgs geometry_msgs
  rosidl_default_runtime)



rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LineSeg.msg"
  "msg/MarkerDetection.msg"
  "action/Test.action"
  "action/InspectWarehouse.action"
  "srv/PathReq.srv"
  DEPENDENCIES geometry_msgs sensor_msgs
)




ament_package()
