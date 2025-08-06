cmake_minimum_required(VERSION 2.8)
project(QtTest LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

include_directories(
    ${CMAKE_SOURCE_DIR}/include
)

find_package(Qt5 REQUIRED COMPONENTS Core Network)

file(GLOB SRC_FILES *.cpp)

add_executable(QtTest
    ${SRC_FILES}
)

target_link_libraries(QtTest PRIVATE
    Qt5::Core
    Qt5::Network
    Audio_Processing
    pthread
)

