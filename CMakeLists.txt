cmake_minimum_required(VERSION 3.16)
project(cpod)

set(CMAKE_CXX_STANDARD 14)

add_executable(cpod main.cpp cpod.cpp stream.cpp)
add_executable(test_mtree test_mtree.cpp)