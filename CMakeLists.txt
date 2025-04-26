cmake_minimum_required(VERSION 3.17)
project(test)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE "Release")

find_package(
    PCL REQUIRED
    COMPONENTS common io filters features segmentation
)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_executable(test test.cc include/ikd-tree/ikd-tree.cc)
target_link_libraries(test ${PCL_LIBRARIES})
