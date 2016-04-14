Graph {#graph}
=====

### a. Design

Graph is a type of data structure that can be used to represent pairwise relations between objects. In this library, a graph is modeled as a collection of vertices and edges. The relations between those concepts are shown as follows.
* Graph
  * Vertex 1
    * Edge 1_1
    * Edge 1_2
    * ...
  * Vertex 2
    * Edge 2_1
    * Edge 2_2
    * ...
  * ...
  * Vertex n
    * Edge n_1
    * Edge n_2
    * ...
    * Edge n_m

A minimal implementation of Graph consists of a list of vertices, each of which has an unique ID and a list of edges. For path finding in the graph, we need to add extra attributes, such as edge cost in Edge and heuristics, flags in Vertex for A* search. These attributes are generic for all graphs.

In different contexts, we usually want to add non-generic attributes to the vertex so that it can be meaningful for the application. For example when we use a graph to represent a square grid, a square cell can be regarded as a vertex, and the connectivities of a cell with its neighbour cells can be represented as edges. In this case, a square cell (vertex) may have attributes such as its location in the grid and its occupancy type (cell filled with obstacle or not). Such attributes can be very different across different applications, thus they are not modeled directly in the "Vertex" data structure. Instead, the "additional information" is packed into a separate object (called a **node** in this design) and we associate a node with a vertex uniquely.

### b. Implementation

There are 3 class templates defined: **Graph**, **Vertex**, **Edge**. The use of template enables us to associate different types of "node" to a vertex, without modifying the code of the aforementioned 3 classes. In other words, the Graph, Vertex and Edge all have a "type", which is determined by the type of node we want to associate with the vertex. With the current implementation, the node has to be defined as a class or struct. An unique ID must be assigned to each node before we use them to construct a graph. In the graph data structure, the vertex has the same ID with the node it's associated with. This is for solely for easy indexing to find one with the other.

Here is an example to use the templates.

I. We first define a node type we want to use for constructing the graph.

~~~
struct ExampleNode{
	ExampleNode(uint64_t id):node_id_(id){}

	const uint64_t node_id_;

	// you can add more attributes here
};
~~~

II. Then we can create a few objects of class ExampleNode

~~~
std::vector<ExampleNode*> nodes;

// create nodes, with id from 0 to 3
for(int i = 0; i < 4; i++) {
	nodes.push_back(new ExampleNode(i));
}
~~~

III. Now use those nodes to construct a graph. Note that the graph is of type ExampleNode in this example.

~~~
// create a graph of type ExampleNode
Graph<ExampleNode> graph;

graph.AddEdge(nodes[0], nodes[1], 1.0);
graph.AddEdge(nodes[0], nodes[2], 1.5);
graph.AddEdge(nodes[1], nodes[2], 2.0);
graph.AddEdge(nodes[2], nodes[3], 2.5);
~~~

IV. Now you've got a graph. You can print all edges of this graph in the following way

~~~
auto all_edges = graph.GetGraphEdges();

for(auto e : all_edges)
	e.PrintEdge();
~~~

You will get the output

~~~
Edge: start - 0 , end - 1 , cost - 1
Edge: start - 0 , end - 2 , cost - 1.5
Edge: start - 1 , end - 2 , cost - 2
Edge: start - 2 , end - 3 , cost - 2.5
~~~

### c. Memory Management

When a Graph object goes out of scope, its destructor function will recycle memory allocated for this its vertices and edges. However, **the graph doesn't recycle memory allocated for the node that each vertex is associated with**. In the square grid example, the graph doesn't assume the square grid also becomes useless when the graph itself is destructed. The **square grid** should be responsible for recycling the memory allocated for its square cells when it becomes of no use. Thus in the above simple example, we will need to do the following operation to free the memory at the end.

~~~
// delete objects of ExampleNode
for(auto e : nodes)
		delete e;
~~~

### d. Notes on Graph

* You may have noticed that when constructing a graph, you don't need to explicitly create objects of "Vertex". By calling member function **AddEdge(src_node, dst_node, cost)** of the graph, vertices are created and associated with the according node internally.
* There are two views of the graph data structure. When constructing the graph (bottom-up view), the nodes are manipulated directly and vertices are handled implicitly. When using the graph (top-down view) for path search, vertices are the the entities you're directly interacting with and the nodes they associate with are probably of less interest. Of course, you can access one from the other easily using their common ID.
* A* performs search on Vertex objects, so the A* algorithm also has a "type". In this implementation, A* search is provided as a member function of Graph. So you don't need to explicitly declare and initialize an A* instance. You can simply perform search on a graph by calling the search function packed in the graph.
* An detailed example of using the graph for path search can be found in "apps/example.cpp". The work flow is shown as follows.

~~~
// create a graph from square grid
Graph<SquareCell>* graph = GraphBuilder::BuildFromSquareGrid(grid,true);

// specify search start and finish vertex
Vertex<SquareCell>* start_vertex = graph->GetVertexFromID(0);
Vertex<SquareCell>* finish_vertex = graph->GetVertexFromID(1);

// perform A* search and get a vector of Vertics as the search result
std::vector<Vertex<SquareCell>*> path = graph->AStarSearch(start_vertex,finish_vertex);
~~~
