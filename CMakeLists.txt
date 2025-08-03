cmake_minimum_required(VERSION 3.10)
project(MyProject LANGUAGES CXX)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Common definitions for all targets
add_definitions(-DWEBRTC_LINUX -DWEBRTC_POSIX -DWEBRTC_NS_FLOAT)

# Global include path for all targets (optional; better to use target_include_directories when possible)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Add subdirectories in correct build order:
add_subdirectory(3rdParty)     # build webrtc_aec (static lib)
add_subdirectory(src)          # build main app
add_subdirectory(test/simple)  # unit test
add_subdirectory(test/qt)      # qt test
