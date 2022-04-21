# RC_Pilot_V2 - Multirotor Flight Control System

Welcome to the  RC_Pilot_V2!
This code is a continuation and rework of the original rc_pilot.
		rc_pilot was the original lighweight flight code for controlling a multirotor developed by 
		Dr. Stawson, Dr. Gaskell and others - https://github.com/StrawsonDesign/rc_pilot
		
It is critical to learn and understand the flightcode before using it. Make sure you know
what you are doing and don't hold me liable for any equipment damage and follow the law!

THIS CODE IS NOT FOR PUBLIC USE. USE AT YOUR OWN RISK. CHECK WITH FAA AND LOCAL UAV REGULATIONS. 

# Installation:
#### Made to run on Beaglebone Blue linux board with the debian image from the official website:
#### https://beagleboard.org/getting-started			- getting started page
#### https://beagleboard.org/latest-images 			- install image from here
#### http://strawsondesign.com/docs/librobotcontrol/ - robot control library (start with manual)

## Pre-INSTAL (LINUX):
#### In teminal, as root, type the following to install all needed packages (CMAKE):
	apt-get update && apt-get install cmake -y && apt-get install build-essential gdb -y

## Project Compilation:
#### Download rc_pilot_v2 folder and create a build directory <build_folder>: 
	mkdir home/debian/build_folder/

### Pre-Compile:
	cmake -S home/debian/rc_pilot_v2/ -B home/debian/build_folder/

### Complie Project:
#### Go into build folder:
	cd home/debian/build_folder/
	make

# Running: 
#### To run the project just call the main execuitable with the proper settings file:
	./MAIN/bin/rc_pilot -s ./MAIN/Settings/SETTINGS_FILE.json

# NOTE:
#### If you need to modify the source code, do it in 
	home/debian/rc_pilot_v2/
#### and repeat all the above compilation steps (cmake and make). Do not modify source files in <build_folder/>, use cmake to update them.
#### You can look into docs/ folder for more information regarding code layout.

#### If you have any questions, please feel free to reach out to me, Yevhenii (Jack) Kovryzhenko at yzk0058@auburn.edu
