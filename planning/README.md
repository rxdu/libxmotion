## Planning

1. Module Dependencies

* common: headers shared by multiple modules
* square_grid: independent
* quadtree: independent
* graph: independent, headers only
* map: depends on square_grid, quadtree, graph
* visualizer: depends on map, square_grid, quadtree, graph

Reference

C++

* http://stackoverflow.com/questions/23488326/c-stdvector-of-references
* http://stackoverflow.com/questions/332030/when-should-static-cast-dynamic-cast-const-cast-and-reinterpret-cast-be-used
