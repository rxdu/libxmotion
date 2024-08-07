## Code organization

Main components of libxmotion (C++)

* platform: applications for robot platforms
* interface: abstract classes to separate the implementation and simplify dependency management
* driver: hardware drivers
* control: motion control algorithms
* planning: decision making and motion planning algorithms
* utilities: tiny 3rd-party libraries and helper/wrappper classes implemented to simplify usage of external libraries
* visualization: data visualization
