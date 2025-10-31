cmake_minimum_required(VERSION 3.10)
project(mtrx3760_lrl_robot_vis)
set(CMAKE_CXX_STANDARD 17)


find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

#---Include .hpp's---
include_directories(include)

install(DIRECTORY include/${PROJECT_NAME}/
          DESTINATION include/${PROJECT_NAME}
          FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")

# Include directories
include_directories(
    external/SDL2/include/SDL2
    external/imgui
    external/imgui/backends
    include/robot_vis
)

# Link SDL2 static library
link_directories(external/SDL2/lib)

# Source files
set(SOURCES
    src/main.cpp
    src/render/render_objs.cpp
    src/socket.cpp
    external/imgui/imgui.cpp
    external/imgui/imgui_draw.cpp
    external/imgui/imgui_tables.cpp
    external/imgui/imgui_widgets.cpp
    external/imgui/backends/imgui_impl_sdl2.cpp
    external/imgui/backends/imgui_impl_sdlrenderer2.cpp
)

ament_export_dependencies(
  rclcpp 
  std_msgs sensor_msgs geometry_msgs nav_msgs
  rosidl_default_runtime)



add_executable(ros2Interface src/ros2_interface_node.cpp)
ament_target_dependencies(ros2Interface rclcpp std_msgs geometry_msgs nav_msgs)
target_link_libraries(ros2Interface)

add_executable(my_app ${SOURCES} )
ament_target_dependencies(my_app rclcpp std_msgs geometry_msgs nav_msgs)
target_link_libraries(my_app SDL2 SDL2main ) 


# --- Install the node executable ---
install(TARGETS ros2Interface
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS my_app
  DESTINATION lib/${PROJECT_NAME}
)



install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()