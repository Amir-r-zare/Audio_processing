cmake_minimum_required(VERSION 3.10)
project(MyProject LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions("-DWEBRTC_LINUX -DWEBRTC_POSIX -DWEBRTC_NS_FLOAT")

# path of header files
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# first, make webrtc library
add_subdirectory(3rdParty)

# build source of project
add_subdirectory(src)

# build test projects
add_subdirectory(test/simple)
add_subdirectory(test/qt)

