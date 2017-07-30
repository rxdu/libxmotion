# SRCL Control

This repository contains a collection of software that are used to develop and test robot related algorithms with physics-based simulation at SRCL.

## 1. Repository structure

* **src** : modules implemented in C++
* **scripts** : python/shell scripts
* **docs** : documentation of this project
* **data** : map, experiment results, logs
* **tools** : tool used for the development

## 2. Install dependencies

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

* Qt5 (Optional)
```
$ sudo apt-get install qt5-default
```

* VTK >= 7.0 (Optional)

Gui module will not be built if VTK is not installed.

Install dependencies first.

```
$ sudo apt-get install qttools5-dev
```
Then use cmake-gui to configure the VTK source with Qt5 support.

Step 1:

Click "Configure", and select the default Unix make tools

- Select "Group_Qt"
- Choose "OpenGL" in VTK_RENDERING_BACKEND
- Select USE_CXX11_FEATURES

Click "Configure" again.

Step 2:

- Choose 5 in VTK_QT_VERSION

Click "Configure" one more time. You should see no red highlighted items and no error information in the output of the configuration.

Setp 3:

Click "Generate" so that cmake will generate a makefile project for you.

```
$ make
$ sudo make install
```

For more information, refer to this official [wiki](http://www.vtk.org/Wiki/VTK/Configure_and_Build).

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
$ cd librav/docs/doxygen
$ doxygen Doxyfile
```

## [Reference]
