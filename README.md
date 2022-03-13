# Library for Robot Navigation

![GitHub Workflow Status](https://github.com/rxdu/robosw/workflows/CMake/badge.svg)

This repository contains a collection of software packages for **robotics experiments and applications**. Many of the components were initially created and maintained in other repositories, such as [librav](https://bitbucket.org/rdu/librav/src/next/), [robotnav](https://github.com/rxdu/robotnav), [imtoolkit](https://github.com/rxdu/imtoolkit). As the number of repositories and the size of my code base keep growing, I find it more and more challenging to keep the code up-to-date and ready for use. So I decide to gradually port and update relevant code from those repositories and put them here at one place for convenience of use and ease of maintenance.

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
$ sudo apt-get install -y libboost-all-dev libeigen3-dev libgsl-dev libtbb-dev libopencv-dev \
    libgl1-mesa-dev libglfw3-dev libcairo2-dev libasio-dev
```

#### Compile code

```
$ mkdir build
$ cmake ..
$ make -j8
```

**Note**: If you get any building issues, please refer to the CI configuration ".github/workflows/cmake.yml" for detailed build steps.
