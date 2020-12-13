# Library for Robot Navigation

![GitHub Workflow Status](https://github.com/rxdu/libnav/workflows/CMake/badge.svg)

This repository contains a collection of software packages for study and research on mobile robot navigation. The modules are built on top of a number of open-source packages. At this moment, most of the available modules developed during my graduate study. I'm in the process of refactoring the existing code and adding new functions as well.

## Repository structure

| Folder |       Description        |
| ------ | ------------------------ |
| cmake  | cmake configuration      |
| data   | experiment results, logs |
| docs   | ocumentation             |
| src    | C++ code                 |

## Build and run

### Requirements

* C++11

```
$ sudo apt-get install -y libeigen3-dev libgsl-dev libtbb-dev libboost-all-dev libopencv-dev
```

### Install dependencies

### Compile code

```
$ mkdir build
$ cmake ..
$ make -j8
```

