#add_subdirectory(include src)
add_library(
    Serial_Tools_lib STATIC
    include/crc16.h
    include/serial_transmit.hpp
    include/serialib.h
    src/serial_transmit.cpp
    src/serialib.cpp)

# at this point we can compile all the sources and headers individually, but they don't see 
# each other, so linking will fail. To fix this, add all the folders to the executable 
# tree (called using variable "Main") so linker can search over it for all #inludes:
target_include_directories(Serial_Tools_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/")
target_include_directories(Serial_Tools_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src/")