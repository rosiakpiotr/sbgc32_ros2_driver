cmake_minimum_required(VERSION 3.4)
project(gimbal)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

add_subdirectory(program)
add_subdirectory(libs/gimbal-driver)