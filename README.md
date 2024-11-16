# xMotion Library

![GitHub Workflow Status](https://github.com/rxdu/robosw/workflows/Main/badge.svg)

This repository contains a collection of software packages for **mobile robot motion planning and motion control**. At
the moment, this library is mainly used for study, research and other experimental purposes. It is not yet ready for
production.

Many of the components were initially created and maintained in a few other repositories, such
as [librav](https://bitbucket.org/rdu/librav/src/next/), [imtoolkit](https://github.com/rxdu/imtoolkit). As the number
of repositories and the size of my code base keep
growing, I find it more and more challenging to keep the code up-to-date and ready for use. So I decide to gradually
port and update relevant code from those repositories and put them here at one place for convenience of use and ease of
maintenance.

## Repository structure

| Folder  | Description         |
|---------|---------------------|
| cmake   | cmake configuration |
| data    | map, results, logs  |
| docs    | documentation       |
| src     | C++ code            |
| python  | Python code         |
| extern  | external libraries  |
| scripts | bash scripts        |

## Build and run

#### Compiler Requirements

* C++11

#### Install dependencies

```
$ sudo apt-get install -y libboost-all-dev libeigen3-dev libgsl-dev libtbb-dev libopencv-dev \
    libgl1-mesa-dev libglfw3-dev libcairo2-dev libasio-dev libyaml-cpp-dev libncurses-dev libevdev-dev 
```

#### Compile code

```
$ mkdir build
$ cmake ..
$ make -j8
```

**Note**: If you get any building issues, please refer to the CI configuration ".github/workflows/cmake.yml" for
the current build steps.

#### Logging configurations

* XLOG_LEVEL: 0 - 6, 0: TRACE, 1: DEBUG, 2: INFO, 3: WARN, 4: ERROR, 5: FATAL, 6: OFF
* XLOG_ENABLE_LOGFILE: 0 or 1
* XLOG_FOLDER: folder to store log files, default folder: `~/.xmotion/log`