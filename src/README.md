## Code organization

Main components of librav (C++)

* **cmake**: cmake cofigurations
* **kernel**
    * common: code shared by multiple modules
    * examples: example apps of basic functions
    * device: linux hardware device drivers
    * control: general control functions
    * decision: general decision making building blocks
    * planning: general motion planning algorithms
    * visualization: core visualization
    * utils: tiny 3rd-party libraries and helper/wrappper classes implemented to simplify usage of external libraries
* **lcmtypes**: type definitions of LCM
* **mcontrol**: platform-dependent motion control algorithms
* **navigation**: high-level planning/navigation modules for applications
* **platforms**
    * quadrotor: applications of quadrotors
    * rc_car: applications of RC cars
    * simulator: V-REP simulator for quadrotor and RC cars
* **third_party**: relative large 3rd-party libraries, could potentially be removed by using system-wide installations
* **unit_tests**: google unit test code
