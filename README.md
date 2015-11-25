# SRCL Control

This repository contains a collection of software that are used to develop and test robot related algorithms with physics-based simulation at SRCL.

## 1. Use Eclipse to build project

* Create a new folder outside of the project root directory

```
$ cd ~/Workspace/srcl_robot_suite/srcl_ctrl/planning
$ mkdir build
$ cd build
```
* Run the command to generate eclipse project from cmake

```
$ cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ../quad_tree/
```
* Import generated project located at build folder into eclipse

You can install an Eclipse plugin from the following source to edit CMAKE files:

```
Name: CMAKE Editor
Location: http://cmakeed.sourceforge.net/eclipse/
```
