# CMakeList.txt : CMake project for HardwareTesting, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8) #sets the minimum required version for this Cmake project

# set the project name
project (HardwareTesting) #project name

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

#For easier management, the project is separated into folders: 
add_subdirectory(libs) #libs/ has all of the required libraries 
#already downloaded, so we can compile it on any linux machine
add_subdirectory(MAIN) #MAIN/ has all the source and header 
#files of the project which are also linking headers from /libs

#note, each subdirectory added with "add_subdirectory()" must 
#contain its own CMakeLists.txt. When CMake is generated, it 
#automatically collects information from all of the individual 
#CMakeLists.txt files. While some projects contain a single giant 
#CMakeLists.txt file all files in a single folder, spreading 
#eveything out and having multiple CMakeLists.txt files makes 
#it easier to track and mannage the project. However, the result 
#is the same.