cmake_minimum_required(VERSION 2.8)
project(AudioProcessing)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../3rdParty/webrtc)
include_directories(${PROJECT_SOURCE_DIR}/include)
add_definitions(-DWEBRTC_LINUX)
file(GLOB SRC_FILES *.cpp)

add_executable(AudioProcessing
    ${SRC_FILES}
)

target_include_directories(AudioProcessing PRIVATE
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(AudioProcessing PRIVATE
    Audio_Processing
    pthread
)
