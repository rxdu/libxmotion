## Code organization

Main components of libxmotion (C++)

* **common**: infrastructure components shared by all other components
* **control**: motion control algorithms
* **driver**: hardware drivers
* **estimation**: state estimation algorithms
* **interface**: abstract classes to separate the implementation and simplify dependency management
* **mapping**: mapping algorithms
* **planning**: decision-making and motion planning algorithms
* **quickviz**: data visualization
