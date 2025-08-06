cmake_minimum_required(VERSION 2.8)
project(SimpleTest LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

include_directories(
    ${CMAKE_SOURCE_DIR}/include
)

file(GLOB SRC_FILES *.cpp)

add_executable(SimpleTest
    ${SRC_FILES}
)

target_link_libraries(SimpleTest PRIVATE
    Audio_Processing
    pthread

)
