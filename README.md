# SRCL Control

This repository contains a collection of software that are used to develop and test robot related algorithms with physics-based simulation at SRCL.

## 1. Repository structure

+ **control** : control code for the robot/simulation
+ **planning** : planning algorithms
+ **(build)** : default location to build the code in planning folder, not tracked in git

## 2. Install Dependencies

* OpenCV
```
$ sudo apt-get install libopencv-dev python-opencv
```

## 3. Use Eclipse to build project

* Create a new folder outside of the project root directory

```
$ cd ~/Workspace/srcl_robot_suite/srcl_ctrl/
$ mkdir -p build/planning_build
$ cd build/planning_build
```
* Run the command to generate eclipse project from cmake

```
$ cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ../../planning
```
* Import generated project located at build folder into eclipse

Similarly you can generate eclipse project for the quadrotor simulation code.

```
$ cd ~/Workspace/srcl_robot_suite/srcl_ctrl/build
$ mkdir quadctrl_build
$ cd quadctrl_build
$ cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ../../control/hummingbird/quad_ctrl/
```

You can install an Eclipse plugin from the following source to edit CMAKE files:

```
Name: CMAKE Editor
Location: http://cmakeed.sourceforge.net/eclipse/
```

## 4. Utils

* Log Analysis: this tool depends on Qt5, so you need to install Qt5 and any Dependencies of Qt5

```
$ sudo apt-get install libglu1-mesa-dev
```

## [Reference]

* [Check integer is power of two](http://www.exploringbinary.com/ten-ways-to-check-if-an-integer-is-a-power-of-two-in-c/)
* [OpenCV function implementation reference](https://github.com/Itseez/opencv/blob/master/modules/imgproc/src/thresh.cpp#L1192)
* https://gehrcke.de/2011/06/reading-files-in-c-using-ifstream-dealing-correctly-with-badbit-failbit-eofbit-and-perror/
* http://stackoverflow.com/questions/7868936/read-file-line-by-line
* http://www.cc.gatech.edu/classes/AY2015/cs4496_spring/Eigen.html
