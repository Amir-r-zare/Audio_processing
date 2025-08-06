cmake_minimum_required(VERSION 2.8)
project(QtTest LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

add_definitions("-DWEBRTC_LINUX -DWEBRTC_POSIX -DWEBRTC_NS_FLOAT")

include_directories(
    ../../include
)

find_package(Qt5 REQUIRED COMPONENTS Core Network)

add_executable(QtTest
    main.cpp
)

target_include_directories(QtTest PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)


target_link_libraries(QtTest PRIVATE
    Qt5::Core
    Qt5::Network
    Audio_Processing
    pthread

)
