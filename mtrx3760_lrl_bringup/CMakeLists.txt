cmake_minimum_required(VERSION 3.8)
project(mtrx3760_lrl_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# CMakeLists.txt example
# ...
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
# ...