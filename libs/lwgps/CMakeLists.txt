#add_subdirectory(include src)
add_library(
    GPS_lib STATIC
    include/lwgps.h
    src/lwgps.c
    )

target_include_directories(GPS_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src") 
target_include_directories(GPS_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
