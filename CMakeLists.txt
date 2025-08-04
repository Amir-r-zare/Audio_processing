cmake_minimum_required(VERSION 3.10)
project(MyProject LANGUAGES CXX)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Common definitions for all targets
add_definitions(-DWEBRTC_LINUX -DWEBRTC_POSIX -DWEBRTC_NS_FLOAT)

# Global include path for all targets
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Add subdirectories in correct build order:
add_subdirectory(3rdParty)
add_subdirectory(src)
add_subdirectory(test/simple)
add_subdirectory(test/qt)
