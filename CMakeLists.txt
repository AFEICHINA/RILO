cmake_minimum_required(VERSION 2.8)
project(RILO)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread -fopenmp")
# set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_CXX_FLAGS_RELEASE "-O0 -Wall -g -pthread -fopenmp")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(OpenMP REQUIRED)

find_package(teaserpp REQUIRED)

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

include_directories(
	include
)

file( GLOB DIR_SRCS ${PROJECT_SOURCE_DIR}/src/*.cpp
					${PROJECT_SOURCE_DIR}/include/*.hpp
					${PROJECT_SOURCE_DIR}/include/ThirdParty/DUtils/*.cpp
					${PROJECT_SOURCE_DIR}/include/ThirdParty/DVision/*.cpp)

add_executable(${PROJECT_NAME}_test test/test.cpp ${DIR_SRCS})
target_link_libraries(${PROJECT_NAME}_test ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${OpenMP_CXX_FLAGS} 
											teaserpp::teaser_registration)