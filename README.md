# Library for Robot Navigation

![GitHub Workflow Status](https://github.com/rxdu/robosw/workflows/CMake/badge.svg)

This repository contains a collection of software packages for study and research on mobile robot navigation. The modules are built on top of a number of open-source packages. Currently most of the available modules were developed during my graduate study. I'm in the process of refactoring the existing code and adding new functions as well.

## Repository structure

| Folder |       Description        |
| ------ | ------------------------ |
| cmake  | cmake configuration      |
| data   | map, results, logs       |
| docs   | documentation            |
| src    | C++ code                 |

## Build and run

#### Compiler Requirements

* C++11

#### Install dependencies

```
$ sudo apt-get install -y libboost-all-dev libeigen3-dev libgsl-dev libtbb-dev libopencv-dev
```

#### Compile code

```
$ mkdir build
$ cmake ..
$ make -j8
```

**Note**: If you get any building issues, please refer to the CI configuration ".github/workflows/cmake.yml" for detailed build steps.
