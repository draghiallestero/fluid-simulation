cmake_minimum_required(VERSION 3.15)

project(fluid-sim)

set(CMAKE_CXX_STANDARD 17 CACHE STRING "")

add_executable(fluid-sim)

target_sources(fluid-sim PRIVATE
  main.cpp
  global_data.cpp
)

target_compile_options(fluid-sim PRIVATE -Wdouble-promotion)

find_package(raylib CONFIG REQUIRED)

target_link_libraries(fluid-sim PRIVATE raylib)

