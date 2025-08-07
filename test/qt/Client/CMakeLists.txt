project(QtClient LANGUAGES CXX)
find_package(Qt5 REQUIRED COMPONENTS Core Network)
file(GLOB SRC_FILES *.cpp)
add_executable(QtClient ${SRC_FILES})
target_include_directories(QtClient PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(QtClient PRIVATE
    Qt5::Core
    Qt5::Network
    Audio_Processing
    pthread
)

