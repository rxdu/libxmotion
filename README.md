# Library for Robot Navigation

![GitHub Workflow Status](https://github.com/rxdu/libnav/workflows/CMake/badge.svg)

This repository contains a collection of software for study and research on mobile robot navigation.

Note: this library is built on top of a number of open-source packages. At this moment, most of the available modules are from my graduate study. I'm in the process of cleaning up the code, redesigning some modules and adding new functions. I use this library mostly for study and experiments on new ideas. I hope it could also be useful for others.

## Repository structure

| Folder |       Description        |
| ------ | ------------------------ |
| cmake  | cmake configuration      |
| data   | experiment results, logs |
| docs   | ocumentation             |
| map    | map files                |
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

