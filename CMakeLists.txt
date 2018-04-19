cmake_minimum_required(VERSION 3.8)
project(bad)

set(CMAKE_CXX_STANDARD 17)

#set(SOURCE_FILES main.cpp lib/blimit.hpp lib/blimit-paper.cpp)
set(SOURCE_FILES main.cpp blimit-paper.cpp blimit.hpp)
add_executable(bad ${SOURCE_FILES})