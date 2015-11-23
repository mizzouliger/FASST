cmake_minimum_required(VERSION 3.3)
project(Thesis)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp Point.cpp Point.h MetricTree.cpp MetricTree.h)
add_executable(Thesis ${SOURCE_FILES})