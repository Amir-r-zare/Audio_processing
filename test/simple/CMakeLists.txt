cmake_minimum_required(VERSION 3.10)
project(SimpleTest LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

file(GLOB SRC_FILES *.cpp)

add_executable(SimpleTest ${SRC_FILES})

# اضافه کردن include path برای هدرهای audio_processing
target_include_directories(SimpleTest PRIVATE ${CMAKE_SOURCE_DIR}/include)

# لینک به wrapper اصلی ما که در src تعریف شد
target_link_libraries(SimpleTest PRIVATE Audio_Processing pthread)






#cmake_minimum_required(VERSION 2.8)
#project(SimpleTest LANGUAGES CXX)

#set(CMAKE_CXX_STANDARD 17)

##include_directories(
##    ${CMAKE_SOURCE_DIR}/include
##)


#file(GLOB SRC_FILES *.cpp)

#add_executable(SimpleTest
#    ${SRC_FILES}
#)

#target_include_directories(SimpleTest PRIVATE ${CMAKE_SOURCE_DIR}/include)
#target_link_libraries(SimpleTest PRIVATE
#    Audio_Processing
#    pthread
#)

