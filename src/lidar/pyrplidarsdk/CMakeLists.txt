cmake_minimum_required(VERSION 3.15...3.27)
project(pyrplidarsdk)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(Python 3.10 COMPONENTS Interpreter Development.Module REQUIRED)

# Try to import all Python components first
execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE NB_DIR)
list(APPEND CMAKE_PREFIX_PATH "${NB_DIR}")
find_package(nanobind CONFIG REQUIRED)

# Include RPLIDAR SDK
set(RPLIDAR_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/rplidar_sdk")

# Add include directories
include_directories(
    ${RPLIDAR_SDK_PATH}/sdk/include
    ${RPLIDAR_SDK_PATH}/sdk/src
)

# Collect common RPLIDAR SDK source files (excluding platform-specific arch directories)
file(GLOB_RECURSE RPLIDAR_COMMON_SOURCES
    ${RPLIDAR_SDK_PATH}/sdk/src/*.c
    ${RPLIDAR_SDK_PATH}/sdk/src/*.cpp
)

# Remove all platform-specific files from common sources
list(FILTER RPLIDAR_COMMON_SOURCES EXCLUDE REGEX ".*arch/(win32|macOS|linux).*")

# Platform-specific settings and sources
if(WIN32)
    # Windows specific sources and definitions
    file(GLOB PLATFORM_SOURCES
        ${RPLIDAR_SDK_PATH}/sdk/src/arch/win32/*.cpp
    )
    add_definitions(-D_WIN32)
elseif(APPLE)
    # macOS specific sources
    file(GLOB PLATFORM_SOURCES
        ${RPLIDAR_SDK_PATH}/sdk/src/arch/macOS/*.cpp
    )
    add_definitions(-D_MACOS)
else()
    # Linux specific sources
    file(GLOB PLATFORM_SOURCES
        ${RPLIDAR_SDK_PATH}/sdk/src/arch/linux/*.cpp
    )
    add_definitions(-D_LINUX)
endif()

# Create nanobind module
nanobind_add_module(
    rplidar_wrapper
    NB_STATIC
    src/rplidar_wrapper.cpp
    ${RPLIDAR_COMMON_SOURCES}
    ${PLATFORM_SOURCES}
)

# Link libraries
if(UNIX AND NOT APPLE)
    target_link_libraries(rplidar_wrapper PRIVATE pthread)
endif()

# Compiler-specific options
target_compile_definitions(rplidar_wrapper PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})

# Install directive for scikit-build-core
install(TARGETS rplidar_wrapper LIBRARY DESTINATION pyrplidarsdk) 