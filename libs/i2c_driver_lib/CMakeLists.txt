#add_subdirectory(include src)
add_library(
    i2c_driver_lib STATIC
    include/i2c_driver.hpp
    src/i2c_driver.cpp
    )

target_include_directories(i2c_driver_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src") 
target_include_directories(i2c_driver_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(i2c_driver_lib PUBLIC robotics_cape)