# TODO List

## Kernel

- [ ] Remove CGAL dependency if possible
- [ ] Add memory leak check tests for all modules in "kernel"
- [ ] Cleanup dependencies on LCM, create a comm package 
- [*] Put all finding folder path function into one place 

## Planning

- [ ] Extend roadmap and traffic sim to support definition of path in osm file
- [ ] Memory management of ReferenceTrajectory/LookaheadZone
- [ ] Monte Carlo simulation for autonomous vehicles
- [ ] Priority queue that supports element priority update
- [ ] RRT and RRT* with Dubins model  
- [*] Check possible memory leak issue of SquareGrid
- [*] Remove the requirement of friendship inside Graph_t for search algorithms
- [*] Remove dependency on OpenCV (now only visualization depends on OpenCV)
- [*] Road network model
- [*] Iterators for Vertex_t and Edge_t

## Control

- [ ] Kalman filter for RC Car

## Simulation

- [*] Finish RC Car simulation interface with LCM/FastRTPS

## Visualization

- [ ] Graph visualization using Cairo
- [*] Better surface visualization

## Misc

- ~~Update the command to invoke uavcan type generator~~
- ~~Fix "ENABLE_LOGGING" macro definition~~
- ~~Gurobi related code is broken under Ubuntu 16.04/Debian Stretch~~