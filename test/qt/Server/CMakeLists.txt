project(QtServer LANGUAGES CXX)
find_package(Qt5 REQUIRED COMPONENTS Core Network)
file(GLOB SRC_FILES *.cpp)
add_executable(QtServer ${SRC_FILES})
target_include_directories(QtServer PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(QtServer PRIVATE
    Qt5::Core
    Qt5::Network
    Audio_Processing
    pthread
)

