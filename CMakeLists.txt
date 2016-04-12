cmake_minimum_required(VERSION 3.3)
project(Thesis)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++14")

set(SOURCE_FILES
        main.cpp
        ControlTree.hpp
        MetricTree.hpp
        FassT.hpp
        FassTGating.hpp
        DistanceMetrics.cpp DistanceMetrics.hpp
        ezOptionParser.hpp
        KdTree.hpp)

add_executable(Thesis ${SOURCE_FILES})
