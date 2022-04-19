Multirotor Flight Control System based on Robotics Cape

# RC_Pilot_V2
Welcome to the  RC_Pilot_V2!
This code is a continuation and rework of the original rc_pilot.
		rc_pilot was the original lighweight flight code for controlling a multirotor developed by 
		Dr. Stawson, Dr. Gaskell and others - https://github.com/StrawsonDesign/rc_pilot
		
It is critical to learn and understand the flightcode before using it. Make sure you know
what you are doing and don't hold me liable for any equipment damage and follow the law!

THIS CODE IS NOT FOR PUBLIC USE. USE AT YOUR OWN RISK. CHECK WITH FAA AND LOCAL UAV REGULATIONS. 

# Installation:
Made to run on Beaglebone Blue linux board with the debian image from the official website:
https://beagleboard.org/getting-started			- getting started page
https://beagleboard.org/latest-images 			- install image from here
http://strawsondesign.com/docs/librobotcontrol/ - robot control library (start with manual)

## Pre-INSTAL (LINUX):
in teminal, as root, type the following to install all needed packages (CMAKE):
apt-get update && apt-get install cmake -y && apt-get install build-essential gdb -y

## Project Compilation:
in terminal create an empty build folder:
mkdir home/debian/ANY_FOLDER_PATH/build/

###pre-compile this project:
cmake -S home/debian/PROJECT_DIR/ -B home/debian/ANY_FOLDER_PATH/build/ 

### go into build folder 
cd home/debian/ANY_FOLDER_PATH/build/

### Complie Project:
make

# Running: 
to run the project just call the main execuitable with the proper settings file:
./MAIN/bin/rc_pilot -s ./MAIN/Settings/SETTINGS_FILE.json

# NOTE:
PROJECT_DIR is rc_pilot_v2 for this project
To change any of the settings, modify the source code in home/debian/PROJECT_DIR/
and repeat all the above compilation steps

Look into docs/ folder for more information

For questions contact me, Yevhenii (Jack) Kovryzhenko - yzk0058@auburn.edu
