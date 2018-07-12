cmake_minimum_required(VERSION 2.6)
set( CMAKE_CXX_FLAGS "-std=c++11")
project(findcircle)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(findcircle main.cpp)
target_link_libraries(findcircle ${OpenCV_LIBS})

#install(TARGETS findcircle RUNTIME DESTINATION bin)
