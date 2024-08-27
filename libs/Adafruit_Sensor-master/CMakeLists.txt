#add_subdirectory(include src)
add_library(
    Adafruit_Sensor_lib STATIC
    include/Adafruit_Sensor.hpp
    src/Adafruit_Sensor.cpp
    )

target_include_directories(Adafruit_Sensor_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src") 
target_include_directories(Adafruit_Sensor_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
