cmake_minimum_required(VERSION 3.5)
project(qt LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions("-DWEBRTC_LINUX -DWEBRTC_POSIX -DWEBRTC_NS_FLOAT")

find_package(Qt5 REQUIRED COMPONENTS Core)

add_executable(qt main.cpp)
target_include_directories(qt PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(qt PRIVATE Qt5::Core webrtc_aec pthread)
