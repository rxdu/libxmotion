# Library for Intelligent Vehicle Navigation

This repository contains a collection of software for study and research on autonomous vehicles.

Note: this library is built on top of a number of open-source packages. At this moment, most of the available modules are from my graduate study. I'm in the process of cleaning up the code, redesigning some modules and adding new functions. My hope is to make this library useful for others interested in mobile robot/intelligent vehicle navigation.

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

* C++17

### Install dependencies

### Compile code

```
$ mkdir build
$ cmake ..
$ make -j8
```

