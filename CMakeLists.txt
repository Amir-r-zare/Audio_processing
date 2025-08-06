cmake_minimum_required(VERSION 3.10)
project(MyProject LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DWEBRTC_LINUX)

add_subdirectory(3rdParty)
add_subdirectory(src)
add_subdirectory(test/simple)
add_subdirectory(test/qt)
