## Code organization

Main components of librav (C++)

* **modules**
    * common: code shared by multiple modules
    * logging: support for logging to files
    * examples: example apps of basic functions
    * device: linux hardware device drivers
    * control: general control functions
    * decision: general decision making building blocks
    * planning: general motion planning algorithms
    * cvdraw: visualization tools using OpenCV
    * utilities: tiny 3rd-party libraries and helper/wrappper classes implemented to simplify usage of external libraries
* **lcmtypes**: type definitions of LCM
* **third_party**: relative large 3rd-party libraries, could potentially be removed by using system-wide installations
* **unit_tests**: google unit test code
