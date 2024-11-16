## Code organization

Main components of libxmotion (C++)

* platform: applications for robot platforms
* interface: abstract classes to separate the implementation and simplify dependency management
* common: infrastructure components shared by all other components
* driver: hardware drivers
* control: motion control algorithms
* planning: decision-making and motion planning algorithms
* visualization: data visualization
