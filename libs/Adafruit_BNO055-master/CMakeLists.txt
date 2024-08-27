#add_subdirectory(include src)
add_library(
    Adafruit_BNO055_lib STATIC
    include/Adafruit_BNO055.hpp    
    src/Adafruit_BNO055.cpp
    )

target_include_directories(Adafruit_BNO055_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src") 
target_include_directories(Adafruit_BNO055_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

#target_link_libraries(Adafruit_BNO055_lib PUBLIC Adafruit_Sensor_lib)
target_link_libraries(Adafruit_BNO055_lib PUBLIC i2c_driver_lib)
