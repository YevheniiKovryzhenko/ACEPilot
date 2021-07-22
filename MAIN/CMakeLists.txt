file(GLOB ALL_MAIN 
"include/*.h" "include/*.hpp" 
"src/*.c" "src/*.cpp")

add_executable(Main ${ALL_MAIN})
target_include_directories(Main PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_include_directories(Main PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(Main PUBLIC GPS_lib)
target_link_libraries(Main PUBLIC robotics_cape)
target_link_libraries(Main PUBLIC lwrb_lib)