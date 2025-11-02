cmake_minimum_required(VERSION 3.8)
project(mtrx3760_lrl_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#---RVIZ grumpy---
set(OpenGL_GL_PREFERENCE LEGACY)

#---Find Dependencies---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED core highgui aruco)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(mtrx3760_lrl_interfaces REQUIRED)

# #---Message Generation---
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/DetectedMarker.msg"
#   DEPENDENCIES geometry_msgs sensor_msgs std_msgs
# )

#---Include .hpp's---
include_directories(include)
include_directories(/usr/include/opencv4)
install(DIRECTORY include/${PROJECT_NAME}/
          DESTINATION include/${PROJECT_NAME}
          FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
# install(FILES include/camera_node.hpp
#         DESTINATION include)

#---Add Launch and Config Files To Share---
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY config
#         DESTINATION share/${PROJECT_NAME})

#---Export Dependancies---
ament_export_dependencies(
  rclcpp 
  tf2 tf2_ros tf2_geometry_msgs
  std_msgs sensor_msgs geometry_msgs
  mtrx3760_lrl_interfaces
  rosidl_default_runtime)


#---Individual Nodes--- 
add_executable(mtrx3760_lrl_camera src/camera_node.cpp)
ament_target_dependencies(mtrx3760_lrl_camera 
                            rclcpp 
                            std_msgs sensor_msgs 
                            tf2 tf2_ros tf2_geometry_msgs
                            OpenCV cv_bridge
                            mtrx3760_lrl_interfaces)
target_include_directories(mtrx3760_lrl_camera PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link messages to camera
# rosidl_get_typesupport_target(cpp_typesupport_target
#   ${PROJECT_NAME} rosidl_typesupport_cpp)
# target_link_libraries(mtrx3760_lrl_camera "${cpp_typesupport_target}")


#---Install Targets---
install(TARGETS
  mtrx3760_lrl_camera
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

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