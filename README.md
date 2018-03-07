# Library for Robotics and Autonomous Vehicles (librav)

This repository contains a collection of software for research on robotics and autonomous vehicles.

## 1. Repository structure

* **src** : modules implemented in C++
* **python**: python code
* **matlab**: matlab code
* **scripts** : shell scripts
* **docs** : documentation of this project
* **data** : map, experiment results, logs

## 2. Install dependencies

Scripts in [this repository](https://github.com/rxdu/devel_setup) can be used to install all required dependencies (and other possibly useful software). For minimal dependency installation, you can follow instructions below.

* Building tools, Git, CMake
```
$ sudo apt-get install build-essential
$ sudo apt-get install git
$ sudo apt-get install cmake
```

* OpenCV
```
$ sudo apt-get install libopencv-dev python-opencv
```

* VTK 7.1.1 (Recommended)

VTK is required for advanced plotting features such as surface plot. It's recommended to compile and install from source. The default version from the package manager in Debian/Ubuntu may not work. Newer version >7.1.1 should also work.

* LCM

Follow instructions on http://lcm-proj.github.io/build_instructions.html

You may need to add the following line to your ~/.bashrc so that the shared library of LCM can be found at runtime.

```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/
```

* Boost (Possibly required by 3rd-praty libraries)

```
$ sudo apt-get install libboost-all-dev
```

* Add environment variables (to use LCM types in Python and Java)
```
# librav
export LIBRAV_HOME=$HOME/Workspace/librav
export PYTHONPATH=$LIBRAV_HOME/src/lcmtypes/python:$LIBRAV_HOME/python:$PYTHONPATH
export CLASSPATH=$LIBRAV_HOME/build/lcmtypes_librav.jar:$CLASSPATH
```

## 3. Use Eclipse to build project

* Create a new folder outside of the project root directory

```
$ cd librav
$ mkdir build
$ cd build
```

* Run the command to generate eclipse project from cmake

```
$ cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ../src
```
* Import generated project located at build folder into eclipse. Make sure "Copy project into workspace" option is unchecked during the import process.

## 4. Build the documentation

Install doxygen if you do not have it on your computer.
```
$ sudo apt-get install doxygen
```

Then you can build the documentation from code.

```
$ cd librav/docs/doxygen
$ doxygen Doxyfile
```

## TODO List

- [ ] Finish plotting of graph and square grid in FastPlot module
- [ ] Fix path_repair module for quadrotors and clean up all relevant code

## [Reference]
