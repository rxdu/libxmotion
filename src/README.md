## Code organization

Main components of librav (C++)

* **dview**: GUI app to start applications 
* **lcmtypes**: type definitions of LCM
* **modules**
    * control: motion control algorithms
    * planning: decision making and motion planning algorithms
    * utilities: tiny 3rd-party libraries and helper/wrappper classes implemented to simplify usage of external libraries
    * visualization: for data visualization
* **unit_tests**: google unit test code
