cmake_minimum_required(VERSION 3.18)
project(hello_world)

add_compile_options(-std=c++17)

list(APPEND CMAKE_MODULE_PATH "/usr/lib/x86_64-linux-gnu/cmake/OpenVDB")
find_package(PCL REQUIRED)
find_package(OpenVDB REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(narrowband src/narrowband.cpp)
target_link_libraries(narrowband OpenVDB::openvdb)

add_executable(pcdtovdb src/pcd_to_vdb.cpp)
target_link_libraries(pcdtovdb OpenVDB::openvdb ${PCL_LIBRARIES})