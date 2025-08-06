cmake_minimum_required(VERSION 2.8)
project(AppCore)

add_definitions("-DWEBRTC_LINUX -DWEBRTC_POSIX -DWEBRTC_NS_FLOAT")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../3rdParty/webrtc)

file(GLOB SRC_FILES *.cpp)

add_executable(VoiceApp
    ${SRC_FILES}
)

target_include_directories(VoiceApp PRIVATE
    ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(VoiceApp PRIVATE
    Audio_Processing
    pthread
)
