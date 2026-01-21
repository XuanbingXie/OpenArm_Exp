cmake_minimum_required(VERSION 3.9)
project(udp)

add_definitions(-Wall  -O3)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(test udp.cpp )

target_include_directories(${PROJECT_NAME}
PUBLIC
include
)

