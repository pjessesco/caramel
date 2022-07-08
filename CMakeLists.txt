cmake_minimum_required(VERSION 3.22)
project(caramel)

set(CMAKE_CXX_STANDARD 20)

include_directories(ext/peanut/Peanut)

add_executable(caramel main.cpp)
