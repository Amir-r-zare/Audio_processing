cmake_minimum_required(VERSION 3.10)
project(SimpleTest LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

file(GLOB SRC_FILES *.cpp)

add_executable(SimpleTest ${SRC_FILES})

target_include_directories(SimpleTest PRIVATE ${CMAKE_SOURCE_DIR}/include)

target_link_libraries(SimpleTest PRIVATE Audio_Processing pthread)
