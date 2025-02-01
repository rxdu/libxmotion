# xMotion Library

![GitHub Workflow Status](https://github.com/rxdu/robosw/workflows/Main/badge.svg)

This repository contains a collection of software packages for **mobile robot motion planning and motion control**. At
the moment, this library is mainly used for study, research and other experimental purposes. It is not yet ready for
production.

Many of the components were initially created and maintained in a few other repositories, such
as [librav](https://bitbucket.org/rdu/librav/src/next/), [imtoolkit](https://github.com/rxdu/imtoolkit). As the number
of repositories and the size of my code base kept growing, I found it more and more challenging to keep the code
up-to-date and ready for use. So I decided to gradually port and update relevant code from those repositories and put
them here at one place for convenience of use and ease of maintenance.

## Repository structure

| Folder      | Description           |
|-------------|-----------------------|
| cmake       | cmake configuration   |
| data        | map, results, logs    |
| docs        | documentation         |
| python      | Python code           |
| scripts     | bash scripts          |
| src         | C++ code              |
| third_party | third-party libraries |

## Build and run

#### Compiler Requirements

* C++11

#### Install dependencies

```
$ sudo apt-get install -y libgl1-mesa-dev \
   libglfw3-dev libcairo2-dev libtbb-dev libasio-dev libboost-all-dev \
   libgsl-dev libeigen3-dev libtbb-dev libopencv-dev libyaml-cpp-dev \
   libncurses-dev libevdev-dev libmodbus-dev libpcl-dev libglm-dev
```

#### Compile code

```
$ mkdir build
$ cmake ..
$ make -j
```

**Note**: If you encounter any building issues, please refer to the CI configuration ".github/workflows/main.yml" for
the up-to-date build steps.

#### Logging configurations

* XLOG_LEVEL: 0 - 6, 0: TRACE, 1: DEBUG, 2: INFO, 3: WARN, 4: ERROR, 5: FATAL, 6: OFF
* XLOG_ENABLE_LOGFILE: 0 or 1
* XLOG_FOLDER: folder to store log files, default folder: `~/.xmotion/log`

