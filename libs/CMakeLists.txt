#Libs folder

#This is a list of all the libraries actually used within the project:
add_subdirectory(lwgps) #lightweight GPS NMEA parser 
add_subdirectory(librobotcontrol-master) #primary libray for working with BBB or any embedded system  
add_subdirectory(lwrb) #required by lwgps
add_subdirectory(json-c) #required by rc_pilot, .json file parser (all in C) 

#note 1: each library is it's own Cmake project which was made into a either static or dynamic library with its own name.
#note 2: there can be more folders and/or files in libs/ but they are not actually visible untill added into Cmake file tree.
#note 3: use lwrb and lwgps as an exaple of how to create a very simple library 