cmake_minimum_required(VERSION 3.8)
project(mtrx3760_lrl_actuator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#---Find Dependencies---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs)
find_package(tf2_ros)

find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mtrx3760_lrl_interfaces REQUIRED)

# Copy necessary files to designated locations in the project
install (
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

#---Include .hpp's---
include_directories(include)
install(DIRECTORY include/${PROJECT_NAME}/
          DESTINATION include/${PROJECT_NAME}
          FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")

#---Add Launch and Config Files To Share---
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})

#---Export Dependancies---
ament_export_dependencies(
  rclcpp rclcpp_action rclcpp_components
  tf2 tf2_ros
  std_msgs sensor_msgs tf2_msgs
  mtrx3760_lrl_interfaces
  rosidl_default_runtime)



#---Setup Action Server---
add_executable(actuator_action_server 
                src/actuator_action_server_node.cpp
                src/controller_coordinator.cpp
                src/controller_variants.cpp)

ament_target_dependencies(actuator_action_server 
  mtrx3760_lrl_interfaces
  tf2_msgs tf2_ros
  rclcpp
  rclcpp_action
  rclcpp_components
)

#---Setup Action Debug Client---
add_executable(actuator_debug_client 
                src/actuator_debug_client_node.cpp )

ament_target_dependencies(actuator_debug_client 
  mtrx3760_lrl_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
)



#---Install Targets---
install(TARGETS
  actuator_action_server
  actuator_debug_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(
  ${PROJECT_NAME}
)


#---Build Testing Config.---
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



ament_package()
