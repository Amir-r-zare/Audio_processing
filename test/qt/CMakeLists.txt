cmake_minimum_required(VERSION 3.10)
project(QtTest LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable Qt-specific features
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
find_package(Qt5 REQUIRED COMPONENTS Core Network)

file(GLOB SRC_FILES *.cpp)

add_executable(QtTest ${SRC_FILES})

target_include_directories(QtTest PRIVATE ${CMAKE_SOURCE_DIR}/include)

target_link_libraries(QtTest PRIVATE
    Qt5::Core
    Qt5::Network
    Audio_Processing
    pthread
)
