cmake_minimum_required(VERSION 2.8)
project(RILO)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(OpenMP REQUIRED)

include_directories(
	include
)

file( GLOB DIR_SRCS ${PROJECT_SOURCE_DIR}/src/*.cpp
					${PROJECT_SOURCE_DIR}/include/*.hpp
					${PROJECT_SOURCE_DIR}/include/ThirdParty/DUtils/*.cpp
					${PROJECT_SOURCE_DIR}/include/ThirdParty/DVision/*.cpp)

add_executable(${PROJECT_NAME}_test test/test.cpp ${DIR_SRCS})
target_link_libraries(${PROJECT_NAME}_test ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OpenMP_CXX_FLAGS})