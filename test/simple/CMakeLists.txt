
cmake_minimum_required(VERSION 3.5)
project(simple LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB SIMPLE_SRC *.cpp)

add_definitions("-DWEBRTC_LINUX -DWEBRTC_POSIX -DWEBRTC_NS_FLOAT")

add_executable(SimpleTest ${SIMPLE_SRC})
target_include_directories(SimpleTest PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(SimpleTest PRIVATE webrtc_aec pthread)

add_executable(simple main.cpp)




#-------------------------------------------------

#cmake_minimum_required(VERSION 3.5)

#project(simple LANGUAGES CXX)

#set(CMAKE_INCLUDE_CURRENT_DIR ON)

#set(CMAKE_AUTOUIC ON)
#set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTORCC ON)

#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

##find_package(Qt5Core)

#file(GLOB SIMPLE_SRC *.cpp)

#add_executable(SimpleTest ${SAMPLE_SRC})
#target_link_libraries(SimpleTest PRIVATE webrtc)
#target_include_directories(SimpleTest PRIVATE ${CMAKE_SOURCE_DIR}/include)

#add_executable(simple
#  main.cpp
#)
##target_link_libraries(simple Qt5::Core)
