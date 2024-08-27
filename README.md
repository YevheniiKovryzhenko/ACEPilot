# ACEPilot - Multirotor Flight Control System

## Hello there 👋

This code is a continuation and rework of the original [rc_pilot](https://github.com/StrawsonDesign/rc_pilot),
which was the original lightweight flight code for controlling a multirotor developed by 
Dr. Stawson, Dr. Gaskell, and others.

This is a native C++ autopilot code I developed at the beginning of my research career in 2020-2022. It implements
cascaded PID control and minimum-snap trajectory guidance. I have not actually used it for many years now, so some instructions
below may no longer work properly.

# Installation:
Made to run on Beaglebone Blue (BBB) Linux board with the Debian image from the official website:\
* A very helpful starting point - [BBB getting started page](https://beagleboard.org/getting-started).
* You must install [official BBB image](https://beagleboard.org/latest-images).
* I recommend you familiarize yourself with the [robot control library](http://strawsondesign.com/docs/librobotcontrol/) (start with the manual).

## Pre-INSTAL (LINUX):
In the terminal, as root, type the following to install all needed packages (CMAKE + gdb + Ninja):
```bash
apt-get update && apt-get install cmake -y && apt-get install build-essential gdb -y && apt-get install ninja-build -y
```

## Project Compilation:
Download the project folder either to your workstation and transfer the files to BBB using scp or [WinSCP (Windows)](https://winscp.net/eng/index.php):
You can place <this_project_name> that you have downloaded from GitHub into (BBB) into
"/home/debian/this_project_name/" directory.
Afterwards, create a build directory <build_folder>:
```bash
mkdir /home/debian/build_folder/
```
	
This would be the location where you want to compile the project to.

### Pre-Compile:
In the terminal, as root, type the following to install all needed packages (CMAKE):
```bash
cmake -S /home/debian/this_project_name/ -B /home/debian/build_folder/
```

### Complie Project:
Go into the build folder:
```bash
cd /home/debian/build_folder/
make
```

# Running the project: 
To run the project, you just need to call the main executable with the proper arguments. To see the list of arguments:
```bash
/home/debian/build_folder/MAIN/rc_pilot -h
```
For now, it only requires attaching a proper <SETTINGS_FILE> as:
	
	/home/debian/build_folder/MAIN/bin/rc_pilot -s /home/debian/build_folder/MAIN/Settings/SETTINGS_FILE.json

# Notes:
## Magnetometer (Compass) Issues
If you have just installed a brand new Debian image, you may run into a problem that the compass is not accessible (rc_calibrate_mag exit with error).
The first thing to try is to run the example script for dmp mode with the magnetometer option enabled:
```bash
rc_test_dmp -m -c -t
```
	
This should print out raw compass readings + filtered and fused Tait Bryan angles. Even if calibration was never performed, this should still work...
Try running calibration again:
```bash
rc_calibrate_mag
```
	
If the same issues persist, try to update everything and repeat the steps above.

## Updating everything:
There are cases when you might need to update all the libraries and even Linux Kernel to remove some of the bugs.
First of all, update all the libraries (as a root):
```bash
apt-get update && apt update && apt-get upgrade -y && apt upgrade -y
```
	
Now go and update the scripts for updating Kernel:
```bash
cd /opt/scripts
git pull
```
Update Kernel (as a root):
```bash
tools/update_kernel.sh
```
	
Reboot for the changes to become effective:
```bash
shutdown -r now
```
You may need to update everything once again since you are now running the most recent Linux Kernel and more recent updates are now available:
```bash
apt-get update && apt update && apt-get upgrade -y && apt upgrade -y
```
	
Some packages might want to be updated manually, for example:
```bash
apt upgrade c9-core-installer
```	
Where you need to manually accept (Y) the update. This should be done for all the packages that have been "kept back".

## Modifying the source code:
It is advised to always work on your machine (Windows/MAC OS Desktop/Laptop) to avoid any possible loss of changes on BBB (it can crash). 
Also, it is much easier to keep track of the new changes if working on your machine and only transferring+compiling on BBB (especially when using
Visual Studio or similar Github-enabled environments).\
If you want to modify the source code from BBB, you need to do it only in <this_project_name> folder and not <build_folder> and repeat the compilation steps
both using "cmake" and "make".

## Documentation:
You can look into docs/ folder for more information regarding code layout and details regarding the control system, estimation, etc.

# Contact
If you have any questions, please feel free to contact me, Yevhenii (Jack) Kovryzhenko, at yzk0058@auburn.edu.

# Credit
This work started during my undergraduate research at [ACELAB](https://etaheri0.wixsite.com/acelabauburnuni) at Auburn University, under the supervision of Dr. Ehsan Taheri. Check out my more recent autopilot-related works like [PX4 Simulink IO Framework](https://github.com/YevheniiKovryzhenko/PX4_SIMULINK_IO_Framework.git), [RRTV TiltWing](https://github.com/YevheniiKovryzhenko/RRTV_TiltWing.git) 
and [Quadrotor_with_FF_Control](https://github.com/YevheniiKovryzhenko/Quadrotor_with_FF_Control.git). 

Feel free to cite any of my earlier papers that directly used this work:
*  Y. Kovryzhenko, “Application of the finite fourier series for smooth motion planning of quadrotors,” Masters Thesis, Auburn University, 2023. Accessed: Jul. 26, 2023. [Online]. Available: https://etd.auburn.edu//handle/10415/8797
*  Y. Kovryzhenko and E. Taheri, “Comparison of minimum-snap and finite fourier series methods for multi-copter motion planning,” in AIAA SCITECH 2022 Forum, San Diego, CA & Virtual: American Institute of Aeronautics and Astronautics, Jan. 2022. doi: 10.2514/6.2022-1085.
* Y. Kovryzhenko and E. Taheri, “Development of Cascaded Control and Path- Planning Algorithms for Autonomous Aerial Vehicles,” Auburn University Journal of Undergraduate Scholarship, p. 4, 2021.

