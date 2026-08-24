cmake_minimum_required(VERSION 3.16)

project(rtab_test LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

find_package(jsoncpp CONFIG REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(RTABMap REQUIRED)

pybind11_add_module(rtabmap_py
    src/rtabmap_py/rtabmap_module.cpp
)

target_include_directories(rtabmap_py
    PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    /usr/local/include/rtabmap-0.23
)

target_link_directories(rtabmap_py
    PRIVATE
    ${PCL_LIBRARY_DIRS}
)

target_link_libraries(rtabmap_py
    PRIVATE
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    rtabmap_core
)

target_compile_definitions(
    rtabmap_py
    PRIVATE
    ${PCL_DEFINITIONS}
)

install(
    TARGETS rtabmap_py
    DESTINATION rtabmap_py
)