## Code organization

Configurations

* **cmake**: cmake cofigurations
* **lcmtypes**: type definitions of LCM

Modules 

* **examples**: example apps of basic functions
* **common**: code shared by multiple modules
* **control**: platform-dependent control algorithms
* **planning**: general motion planning algorithms
* **gtests**: google test code
* **vis**: basic visualization

Libraries

* **libraries**: relative large 3rd-party libraries, could potentially be removed by using system-wide installations
* **utility**: small 3rd-party libraries and helper/wrappper classes implemented to simplify usage of external libraries

Platforms

* **simulator**: V-REP simulatior for quadrotor and RC cars
* **quadrotor**: applications of quadrotors
* **rc_car**: applications of RC cars

## LCM Channels

### Quadrotor

* quad_data/quad_transform: LIDAR to quadrotor base and base to world transformation
* quad_data/laser_scan_points: laser scan points from onboard LIDAR
* quad_data/system_time: system time of the quadrotor

### Planner

* quad_planner/goal_keyframe_set: keyframes generated from planner
* quad_planner/trajectory_polynomial: optimized polynomial trajectory
