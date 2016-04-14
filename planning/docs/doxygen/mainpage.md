Main Page                         {#mainpage}
=========

## Overview

The working environment of a robot is usually represented as a map. To find a path for the robot to traverse the environment, we need to extract information from the map and use proper data structures to represent the workspace. A variety of approaches have been proposed by researchers to find a path from the workspace which can satisfy specified requirements.

### 1. Planning Based on Discrete Search

The workspace can be represented by smaller, connected areas so that discrete search algorithms can be used for path planning. As shown in the following figure, one needs to choose a proper method to decompose the workspace first and then use a graph to represent the pairwise relations between neighbour areas. With the graph, algorithms like A* can be performed to find a sequence of areas that connects the starting and finishing points.

![Work Flow](@ref workflow.png)

Currently there are two methods provided to decompose the workspace: square grid and quadtree.

### 2. Sample-Based Motion Planning

## Modules

* Square grid
* Quadtree
* [Graph](@ref graph)

## Known Issues

* A* search algorithm currently only works with nodes that have attribute "location_". This attribute is used to calculate heuristic cost. A more general method may need to be implemented in the future.
