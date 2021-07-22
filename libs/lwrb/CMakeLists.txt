#add_subdirectory(include src)
add_library(
    lwrb_lib STATIC
    include/lwrb.h
    src/lwrb.c
    )

target_include_directories(lwrb_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src") 
target_include_directories(lwrb_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
