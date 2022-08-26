# RC_Pilot_V2 - Multirotor Flight Control System

Welcome to the  RC_Pilot_V2!\
This code is a continuation and rework of the original rc_pilot.
rc_pilot was the original lighweight flight code for controlling a multirotor developed by 
Dr. Stawson, Dr. Gaskell and others - https://github.com/StrawsonDesign/rc_pilot
		
It is critical to learn and understand the flightcode before using it. Make sure you know
what you are doing and don't hold me liable for any equipment damage and follow the law!

THIS CODE IS NOT FOR PUBLIC USE. USE AT YOUR OWN RISK. CHECK WITH FAA AND LOCAL UAV REGULATIONS. 

# Installation:
Made to run on Beaglebone Blue linux board with the debian image from the official website:\
https://beagleboard.org/getting-started			- getting started page\
https://beagleboard.org/latest-images 			- install image from here\
http://strawsondesign.com/docs/librobotcontrol/ - robot control library (start with manual)

## Pre-INSTAL (LINUX):
In teminal, as root, type the following to install all needed packages (CMAKE + gdb + Ninja):

	apt-get update && apt-get install cmake -y && apt-get install build-essential gdb -y && apt-get install ninja-build -y

## Project Compilation:
Download the project folder either to you workstation and transfer the files to BBB using scp or winSCP (Windows):\
https://winscp.net/eng/index.php \
You can place <this_project_name> that you have downloaded from github into (BBB) into
"/home/debian/this_project_name/" directory.
Afterwards, create a build directory <build_folder>:

	mkdir /home/debian/build_folder/
	
This would be the location where you want to compile the project to.

### Pre-Compile:
In teminal, as root, type the following to install all needed packages (CMAKE):

	cmake -S /home/debian/this_project_name/ -B /home/debian/build_folder/

### Complie Project:
Go into build folder:

	cd /home/debian/build_folder/
	make

# Running: 
To run the project just call the main execuitable with the proper arguments. To see the list of arguments:

	/home/debian/build_folder/MAIN/rc_pilot -h
	
For now, it only requires attaching a proper <SETTINGS_FILE> as:
	/home/debian/build_folder/MAIN/bin/rc_pilot -s /home/debian/build_folder/MAIN/Settings/SETTINGS_FILE.json

If you have any questions, please feel free to reach out to me, Yevhenii (Jack) Kovryzhenko at:

	yzk0058@auburn.edu

# Notes:
## Magnetometer (Compass) Issues
If you have just installed a brand new debian image, you may run into a problem that compass is not accessible (rc_calibrate_mag exit with error).
The first thing to try is to run the example script for dmp mode with magnetometer option enabled:

	rc_test_dmp -m -c -t
	
This should print out raw compass readings + filtered and fused Tait Bryan angles. Even if calibration was never performed, this should still work...
Try running calibration again:

	rc_calibrate_mag
	
If the same issues persist, try to update everything and repeat the steps above.

## Updating everything:
There are cases when you might need to update all the libraries and even Linux Kernel to remove some if the bugs.
First of all, update all the libraries (as a root):

	apt-get update && apt update && apt-get upgrade -y && apt upgrade -y
	
Now go and update the scripts for updating Kernel:

	cd /opt/scripts
	git pull

Update Kernel (as a root):

	tools/update_kernel.sh
	
Reboot for the changes to become effective:

	shutdown -r now
	
You may need to update everything once again, since you are now running the most recent Linux Kernel and more recent updates are now available:

	apt-get update && apt update && apt-get upgrade -y && apt upgrade -y
	
Some packages might want to be updated manually, for example:

	apt upgrade c9-core-installer
	
Where you need to manually accept (Y) the update. This should be done for all of the packeges that have been "kept back".

## Modifying the source code:
It is advised to always work on you own machine (Windows/MAC OS Desktop/Laptop) to avoid any possibe loss of changes on BBB (it can crash). 
Also, it is much easier to keep track of the new changes if working on you machine and only transferring+compiling on BBB (especially when using
Visual Studio or similar Github-enabled environments).\
If you want to modify the source code from BBB, you need to do it only in <this_project_name> folder and not <build_folder> and repeat compilation steps
both using "cmake" and "make".

## Documentation:
You can look into docs/ folder for more information regarding code layout and details regarding the control system, estimation, etc.
