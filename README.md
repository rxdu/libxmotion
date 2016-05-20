# SRCL Control

This repository contains a collection of software that are used to develop and test robot related algorithms with physics-based simulation at SRCL.

## 1. Repository structure

+ **control** : control code for the robot/simulation
+ **planning** : planning algorithms

## 2. Install Dependencies

* OpenCV
```
$ sudo apt-get install libopencv-dev python-opencv
```

* Boost (required by the Qt GUI)
```
$ sudo apt-get install libboost-all-dev
```

## 3. Use Eclipse to build project

* Create a new folder outside of the project root directory

```
$ cd <your-preferred-workspace>
$ mkdir srcl_ctrl_build
$ cd srcl_ctrl_build
```
* Run the command to generate eclipse project from cmake

```
$ cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ~/Workspace/srcl_robot_suite/srcl_ctrl/
```
* Import generated project located at build folder into eclipse

You can install an Eclipse plugin from the following source to edit CMAKE files:

```
Name: CMAKE Editor
Location: http://cmakeed.sourceforge.net/eclipse/
```

## 4. Build the Documentation

```
$ cd srcl_ctrl/docs/doxygen
$ doxygen Doxyfile
```


## [Reference]

* [Check integer is power of two](http://www.exploringbinary.com/ten-ways-to-check-if-an-integer-is-a-power-of-two-in-c/)
* [OpenCV function implementation reference](https://github.com/Itseez/opencv/blob/master/modules/imgproc/src/thresh.cpp#L1192)
* https://gehrcke.de/2011/06/reading-files-in-c-using-ifstream-dealing-correctly-with-badbit-failbit-eofbit-and-perror/
* http://stackoverflow.com/questions/7868936/read-file-line-by-line
* http://www.cc.gatech.edu/classes/AY2015/cs4496_spring/Eigen.html
