cmake_minimum_required(VERSION 2.8)
project(AudioProcessing_cm)

include_directories(${CMAKE_SOURCE_DIR}/3rdParty/webrtc)
include_directories(${CMAKE_SOURCE_DIR}/include)

add_definitions("-DWEBRTC_LINUX -DWEBRTC_POSIX -DWEBRTC_NS_FLOAT")

file(GLOB SRC_FILES *.cpp)

add_executable(AudioProcessing_cm
    ${SRC_FILES}
)

#target_include_directories(AudioProcessing PRIVATE
#    ${PROJECT_SOURCE_DIR}
#    ${PROJECT_SOURCE_DIR}/include
#)

target_link_libraries(AudioProcessing_cm PRIVATE
    Audio_Processing
    pthread
)


