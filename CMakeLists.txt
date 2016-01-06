cmake_minimum_required(VERSION 3.3)
project(Thesis)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

set(SOURCE_FILES
        main.cpp
        Point.cpp Point.hpp
        IMetricTree.hpp
        ControlTree.hpp
        MetricTree.hpp
        GatedMetricTree.hpp
        LooselyBoundedMetricTree.hpp
        BoundedMetricTree.hpp
        TriangleUtils.hpp TriangleUtils.cpp)

add_executable(Thesis ${SOURCE_FILES})
