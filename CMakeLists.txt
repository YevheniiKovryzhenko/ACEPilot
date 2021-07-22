# CMakeList.txt : CMake project for HardwareTesting, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

# set the project name
project (HardwareTesting)

add_subdirectory(libs)
add_subdirectory(MAIN)