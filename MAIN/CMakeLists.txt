#making all headers and sources global under variable "ALL_MAIN"
file(GLOB ALL_MAIN 
"include/mod/*.h" "include/mod/*.hpp"
"include/core/*.h" "include/core/*.hpp"
"include/io/*.h" "include/io/*.hpp"
"include/opt/*.h" "include/opt/*.hpp"
"src/core/*.c" "src/core/*.cpp"
"src/mod/*.c" "src/mod/*.cpp"
"src/io/*.c" "src/io/*.cpp"
"src/opt/*.c" "src/opt/*.cpp")

add_executable(Main ${ALL_MAIN}) #this tells the compiler what to compile (ALL_MAIN files) 
# and calls them executables.

# at this point we can compile all the sources and headers individually, but they don't see 
# each other, so linking will fail. To fix this, add all the folders to the executable 
# tree (called using variable "Main") so linker can search over it for all #inludes:
target_include_directories(Main PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/core")
target_include_directories(Main PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src/core")
target_include_directories(Main PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/mod")
target_include_directories(Main PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src/mod")
target_include_directories(Main PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/io")
target_include_directories(Main PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src/io")
target_include_directories(Main PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/opt")
target_include_directories(Main PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src/opt")
#Link all the outside libraries using in this project (from libs folder):
target_link_libraries(Main PUBLIC GPS_lib)
target_link_libraries(Main PUBLIC robotics_cape)
target_link_libraries(Main PUBLIC lwrb_lib)
target_link_libraries(Main PUBLIC json-c)

#This forces to use threads library:
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(Main PRIVATE Threads::Threads) #just including pthread.h is not enough

#Make sure to copy "Settings"" folder to the build directory:
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/Settings" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/")
