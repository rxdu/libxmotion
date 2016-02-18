## Planning

1. Module Dependencies

* common: headers shared by multiple modules
* square_grid: independent
* quadtree: independent
* graph: independent, headers only
* map: depends on square_grid, quadtree, graph
* visualizer: depends on map, square_grid, quadtree, graph
