#making all headers and sources global and attachin to Cmake variable "ALL_MAIN"
file(GLOB ALL_MAIN 
"include/core/*.h" "include/core/*.hpp"
"include/controllers/*.h" "include/controllers/*.hpp"
"include/mod/*.h" "include/mod/*.hpp"
"include/io/*.h" "include/io/*.hpp"
"include/opt/*.h" "include/opt/*.hpp"
"src/core/*.c" "src/core/*.cpp"
"src/controllers/*.c" "src/controllers/*.cpp"
"src/mod/*.c" "src/mod/*.cpp"
"src/io/*.c" "src/io/*.cpp"
"src/opt/*.c" "src/opt/*.cpp")

add_executable(rc_pilot ${ALL_MAIN}) #this tells the compiler what to compile (ALL_MAIN files) 
# and calls them executables.

# Next, tell the compiler to copy ALL_MAIN files to the whatever directory it will be compiling in.
# This is to make sure, when compiling on remote machine, files are actually present at the moment 
# when compilation occurs. They will be deleted after the Main program has been successfully compiled.
source_group("ALL_FILES" FILES ${ALL_MAIN})
# note, the CMakeFile/ folder will be created in the ${CMAKE_CURRENT_BINARY_DIR} of the target system
# and will contain all .o files. Not Source Code, but each and every source filed compiled 
# individually.

# at this point we can compile all the sources and headers individually, but they don't see 
# each other, so linking will fail. To fix this, add all the folders to the executable 
# tree (called using variable "Main") so linker can search over it for all #inludes:
target_include_directories(rc_pilot PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/core")
target_include_directories(rc_pilot PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src/core")
target_include_directories(rc_pilot PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/controllers")
target_include_directories(rc_pilot PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src/controllers")
target_include_directories(rc_pilot PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/mod")
target_include_directories(rc_pilot PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src/mod")
target_include_directories(rc_pilot PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/io")
target_include_directories(rc_pilot PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src/io")
target_include_directories(rc_pilot PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/opt")
target_include_directories(rc_pilot PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src/opt")
# note, we can do all the above lines by droping a separate CMakeLists.txt into each of the subfolder
# and adding respective header and source file along with the path to the Cmake file tree, but this
# allows reducing everyhting to a one general CMakeLists.txt and keeping everything important for the 
# MAIN project in one place. 

# Link all the outside libraries used in this project (from libs folder):
target_link_libraries(rc_pilot PUBLIC GPS_lib)
target_link_libraries(rc_pilot PUBLIC robotics_cape)
target_link_libraries(rc_pilot PUBLIC lwrb_lib)
target_link_libraries(rc_pilot PUBLIC json-c)

# This forces to use threads library:
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(rc_pilot PRIVATE Threads::Threads) #just including pthread.h is not enough

# At this point, everything will compile as it should. However, rc_pilot requires a settings file
# to be attached at run time, but the destination/build folder has no source files or any other
# original file. We can force copy the any folder and/or file with all its content from 
# ${CMAKE_CURRENT_SOURCE_DIR} to ${CMAKE_CURRENT_BINARY_DIR} and then launch the debug with the 
# extra parameters (the later is not configured with CMake but using Visual Studio settings.
# Make sure to copy "Settings"" folder to the build directory:
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/Settings" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/")

### Only Visual Studio Related ###
# To add any extra options for a file at run right click on the file (main.cpp), select 
# "Open Debug and Launch Settings" and change the file accodingly. For example, if we wanted to 
# exectute our program manually, we'd go to the ${CMAKE_CURRENT_BINARY_DIR} and typed:
# ./Main -s Settings/YK_settings.json		in the console.
# To do this automatically, each time we run it using Visual Studio, modify the line:
#	"args": [],														such that it is:
#	"args": ["-s", "Settings/YK_settings_X.json"],