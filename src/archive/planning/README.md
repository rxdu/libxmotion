## Planning

(TO BE UPDATED)

### 1. Module Dependencies

* common: headers shared by multiple modules
* square_grid: independent
* quadtree: independent
* graph: independent, headers only
* map: depends on square_grid, quadtree, graph
* visualizer: depends on map, square_grid, quadtree, graph

### 2. Data Structures

* square grid: represents a 2D space in a uniform way
* quadtree: represents a 2D space with non-uniform cells
* map: since a map in the image format cannot be used for path planning directly, this data structure stores the world information in the image using a square grid or a quadtree structure
* graph: a graph consists of a set of vertices and edges, representing the connectivities among entities. A graph can be constructed from a square grid or quadtree structure.

Reference

C++

* http://stackoverflow.com/questions/23488326/c-stdvector-of-references
* http://stackoverflow.com/questions/332030/when-should-static-cast-dynamic-cast-const-cast-and-reinterpret-cast-be-used
* http://stackoverflow.com/questions/14528902/class-template-with-both-pointer-type-and-regular-type
* http://stackoverflow.com/questions/14466620/c-template-specialization-calling-methods-on-types-that-could-be-pointers-or/14466705
