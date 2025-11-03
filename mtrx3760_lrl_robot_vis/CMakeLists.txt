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
find_package(mtrx3760_lrl_interfaces REQUIRED)
find_package(mtrx3760_lrl_warehouse_inspection REQUIRED)


set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "$ORIGIN/../")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


# Include directories
include_directories(
    external/SDL2/include/SDL2
    external/SDL_ttf-main
    external/libhv/include/hv
    external/imgui
    external/imgui/backends
    include
)

# Link SDL2 static library
link_directories(external/SDL2/lib)
link_directories(external/libhv/lib)
add_subdirectory(external/SDL_ttf-main)



# Source files
set(SOURCES
    src/main.cpp
    src/render/render_objs.cpp
    src/interface/socket.cpp
    src/visualiser.cpp
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
  rosidl_default_runtime
)



add_executable(ros2Interface src/interface/ros2_interface_node.cpp src/interface/socket.cpp)
ament_target_dependencies(ros2Interface rclcpp 
                          std_msgs geometry_msgs sensor_msgs nav_msgs 
                          mtrx3760_lrl_interfaces mtrx3760_lrl_warehouse_inspection)
                          
                          
target_link_libraries(ros2Interface hv_static)

add_executable(robot_interface ${SOURCES} )
ament_target_dependencies(robot_interface rclcpp std_msgs geometry_msgs nav_msgs)

target_link_libraries(robot_interface SDL2 SDL2main SDL2_ttf hv_static) 




# --- Install the node executable ---
install(TARGETS ros2Interface
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS robot_interface
  DESTINATION lib/${PROJECT_NAME}
)



install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()