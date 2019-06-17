cmake_minimum_required(VERSION 3.2)
project(pclBB_view)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(PCL REQUIRED)

include_directories(
${PCL_INCLUDE_DIRS}
)
add_executable(pcl_bb_view pcl_bb_viewer.cpp)
target_link_libraries(pcl_bb_view ${PCL_LIBRARIES})