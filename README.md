# SRCL Control

This repository contains a collection of software that are used to develop and test robot related algorithms with physics-based simulation at SRCL.

## 1. Repository structure

* **pc** : contains software run on a general-purpose computer
    + control : control code for the robot/simulation
    + planning : planning algorithms
    + third_party : third-party libraries

* **mcu** : contains code run on the flight control microcontroller
    + asctec_sdk : sdk for asctec quadrotors
    + px4 : code for px4 compatible flight control boards

* **tools** : tool used for the development
* **docs** : documentation of this project

## 2. Install dependencies

* OpenCV
```
$ sudo apt-get install libopencv-dev python-opencv
```

* LCM

Follow instructions on http://lcm-proj.github.io/build_instructions.html

You may need to add the following line to your ~/.bashrc so that the shared library of LCM can be found at runtime.

```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/
```

* Qt5
```
$ sudo apt-get install qt5-default
```

* Boost (required by the Qt GUI)
```
$ sudo apt-get install libboost-all-dev
```

## 3. Use Eclipse to build project

* Create a new folder outside of the project root directory

```
$ cd srcl_ctrl
$ mkdir build
$ cd build
```
* Run the command to generate eclipse project from cmake

```
$ cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ../pc
```
* Import generated project located at build folder into eclipse

You can install an Eclipse plugin from the following source to edit CMAKE files:

```
Name: CMAKE Editor
Location: http://cmakeed.sourceforge.net/eclipse/
```

## 4. Build the documentation

Install doxygen if you do not have it on your computer.
```
$ sudo apt-get install doxygen
```

Then you can build the documentation from code.

```
$ cd srcl_ctrl/docs/doxygen
$ doxygen Doxyfile
```

## 5. Changes made to third-party libraries

* eigen: unchanged
* octomap-1.8.0: a. changed default output directory from "CMAKE_SOURCE_DIR" to "CMAKE_BINARY_DIR" in the top-level CMakeLists; b. set "CMAKE_INSTALL_PREFIX" to be "CMAKE_BINARY_DIR"; c. commented out conditions for "add_dependencies()" for octovis and dynamicsEDT3D
* g3log_srcl: refer to "change_forl_srcl.txt" inside the library folder

## [Reference]

* [Check integer is power of two](http://www.exploringbinary.com/ten-ways-to-check-if-an-integer-is-a-power-of-two-in-c/)
* [OpenCV function implementation reference](https://github.com/Itseez/opencv/blob/master/modules/imgproc/src/thresh.cpp#L1192)
* https://gehrcke.de/2011/06/reading-files-in-c-using-ifstream-dealing-correctly-with-badbit-failbit-eofbit-and-perror/
* http://stackoverflow.com/questions/7868936/read-file-line-by-line
* http://www.cc.gatech.edu/classes/AY2015/cs4496_spring/Eigen.html
* http://johnnado.com/cmake-directory-variables/
