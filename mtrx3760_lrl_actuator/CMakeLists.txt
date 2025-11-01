cmake_minimum_required(VERSION 3.8)
project(mtrx3760_lrl_actuator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Find dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mtrx3760_lrl_interfaces REQUIRED)

# --- Include ---
include_directories(include)
ament_export_include_directories(include)

# --- Libraries ---
add_library(actuator_client src/actuator_client.cpp)
ament_target_dependencies(actuator_client
  rclcpp rclcpp_action mtrx3760_lrl_interfaces std_msgs
)

# Export library and headers so other packages can use them
ament_export_libraries(actuator_client)
ament_export_include_directories(include)

# --- Executables ---
add_executable(actuator_action_server
  src/actuator_action_server_node.cpp
  src/controller_coordinator.cpp
  src/controller_variants.cpp
)
ament_target_dependencies(actuator_action_server
  rclcpp rclcpp_action rclcpp_components
  mtrx3760_lrl_interfaces tf2_msgs tf2_ros
)

# --- Install targets (FIXED) ---
# Install the library to the standard 'lib' folder so it can be found by other packages
install(TARGETS 
  actuator_client 
  DESTINATION lib
)

# Install the executable to the package-specific folder
install(TARGETS
  actuator_action_server
  DESTINATION lib/${PROJECT_NAME}
)

# --- Install headers ---
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# --- Install launch/config ---
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# --- Export dependencies ---
ament_export_dependencies(
  rclcpp rclcpp_action rclcpp_components
  tf2 tf2_ros
  std_msgs sensor_msgs tf2_msgs
  mtrx3760_lrl_interfaces
  rosidl_default_runtime
)

# --- Testing ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
