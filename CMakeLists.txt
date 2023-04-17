cmake_minimum_required(VERSION 2.6)
project(rrt)
# c++ 11
set(CMAKE_CXX_FLAGS "-std=c++17")
# 调试程序
set(CMAKE_BUILD_TYPE "Debug")
 
find_package(OpenCV 4.7.0 REQUIRED) 	# find opencv4.7 installed path
include_directories(OpenCV_INCLUDE_DIRS)	# load opencv header files
include_directories(include)
add_executable(rrt src/main.cpp src/rrt.cpp)	# create exe file
target_link_libraries(rrt ${OpenCV_LIBS})	# link llib files to exe
