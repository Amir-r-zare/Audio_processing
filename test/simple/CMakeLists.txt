cmake_minimum_required(VERSION 2.8)
project(SimpleTest LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#add_definitions("-DWEBRTC_LINUX -DWEBRTC_POSIX -DWEBRTC_NS_FLOAT")

add_executable(SimpleTest
    main.cpp
)

target_include_directories(SimpleTest PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(SimpleTest PRIVATE
    Audio_Processing
    pthread

)
