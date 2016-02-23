cmake_minimum_required(VERSION 3.3)
project(Thesis)

#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

set(SOURCE_FILES
        main.cpp
        ISearchTree.hpp
        ControlTree.hpp
        MetricTree.hpp
        GatedTree.hpp
        BoundedTree.hpp
        FasstTree.hpp
        TriangleUtils.hpp TriangleUtils.cpp
        DistanceMetrics.cpp DistanceMetrics.hpp)

add_executable(Thesis ${SOURCE_FILES})
