cmake_minimum_required(VERSION 2.8)
project(AudioProcessing_cm)

include_directories(${CMAKE_SOURCE_DIR}/3rdParty/webrtc)
include_directories(${CMAKE_SOURCE_DIR}/include)

add_definitions("-DWEBRTC_LINUX -DWEBRTC_POSIX -DWEBRTC_NS_FLOAT")

file(GLOB SRC_FILES *.cpp)

add_library(AudioProcessing_cm
    ${SRC_FILES}
)

target_link_libraries(AudioProcessing_cm PRIVATE
    Audio_Processing
    pthread
)


