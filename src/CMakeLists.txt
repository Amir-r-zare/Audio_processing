add_executable(MyApp ${SRC_FILES})

target_include_directories(MyApp PRIVATE
    ${PROJECT_SOURCE_DIR}/include
)

add_definitions("-DWEBRTC_LINUX -DWEBRTC_POSIX -DWEBRTC_NS_FLOAT")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../3rdParty/webrtc)

target_link_libraries(MyApp PRIVATE
    webrtc_aec
    pthread
)

file(GLOB SRC_FILES *.cpp)

add_executable(MyProject
    main.cpp
)
