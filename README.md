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

* **sim** : simulation scene files (V-REP)
* **tools** : tool used for the development
* **docs** : documentation of this project

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

* Qt5
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
$ cd srcl_ctrl
$ mkdir build
$ cd build
```

* Run the command to generate eclipse project from cmake

```
$ cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ../pc
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
$ cd srcl_ctrl/docs/doxygen
$ doxygen Doxyfile
```

## [Reference]

* [Check integer is power of two](http://www.exploringbinary.com/ten-ways-to-check-if-an-integer-is-a-power-of-two-in-c/)
* [OpenCV function implementation reference](https://github.com/Itseez/opencv/blob/master/modules/imgproc/src/thresh.cpp#L1192)
* https://gehrcke.de/2011/06/reading-files-in-c-using-ifstream-dealing-correctly-with-badbit-failbit-eofbit-and-perror/
* http://stackoverflow.com/questions/7868936/read-file-line-by-line
* http://www.cc.gatech.edu/classes/AY2015/cs4496_spring/Eigen.html
* http://johnnado.com/cmake-directory-variables/

C++ template

* http://accu.org/index.php/journals/442
* http://stackoverflow.com/questions/25119444/c-specialize-template-class-function-without-duplicating-code
* http://en.cppreference.com/w/cpp/language/partial_specialization
* http://stackoverflow.com/questions/7116610/avoiding-duplication-of-function-definitions-in-template-specializations
* http://eli.thegreenplace.net/2014/sfinae-and-enable_if/
* https://www.informit.com/guides/content.aspx?g=cplusplus&seqNum=50
