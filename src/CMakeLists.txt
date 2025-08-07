cmake_minimum_required(VERSION 3.10)

# این پروژه ماژول wrapper رو می‌سازه
project(AudioProcessingWrapper LANGUAGES CXX)

# هدرهای پروژه ما
include_directories(${CMAKE_SOURCE_DIR}/include)

# وابستگی به WebRTC
include_directories(${CMAKE_SOURCE_DIR}/3rdParty/webrtc)

# ساخت لایبرری استاتیک wrapper ما
add_library(Audio_Processing STATIC audio_processing.cpp)

# لینک به لایبرری WebRTC که در 3rdParty ساخته شده
target_link_libraries(Audio_Processing PRIVATE WebRTC_Audio pthread)

# برای اینکه بقیه (مثل تست) بتونن هدرهای ما رو ببینن
target_include_directories(Audio_Processing PUBLIC ${PROJECT_SOURCE_DIR}/include)




#cmake_minimum_required(VERSION 2.8)
#project(AudioProcessing_cm)

#include_directories(${CMAKE_SOURCE_DIR}/3rdParty/webrtc)
#include_directories(${CMAKE_SOURCE_DIR}/include)

#add_definitions("-DWEBRTC_LINUX -DWEBRTC_POSIX -DWEBRTC_NS_FLOAT")

#file(GLOB SRC_FILES *.cpp)

#target_include_directories(Audio_Processing PUBLIC ${PROJECT_SOURCE_DIR}/include)


#target_link_libraries(AudioProcessing_cm PRIVATE
#    Audio_Processing
#    pthread
#)

#target_include_directories(Audio_Processing PUBLIC ${PROJECT_SOURCE_DIR}/include)
