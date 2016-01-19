/*
 * graph.h
 *
 *  Created on: Dec 9, 2015
 *      Author: rdu
 *
 *  Description:
 *  	1. This file defines template classes of the graph structure, for creating graphs
 *  	from trees. The basic elements include:
 *  	 	a. Vertex
 *  	 	b. Edge
 *  	 	c. Graph
 *  	A vertex is associated with a tree node; an edge connects two vertices; a graph is
 *  	a collections of vertices and the edges of each vertex.
 *
 *  	2. The graph uses unique IDs to index its vertices, so it's required that each tree
 *  	node used to construct the graph should have a unique ID (named "node_id_").
 *
 *  	3. A visualized illustration of the graph structure
 *
 *  	Graph "G":
 *  		Vertex V1 - Edge 1 to vertex V_x1
 *  				  - Edge 2 to vertex V_x2
 *  				  			...
 *  	    Vertex V2 - Edge 1 to vertex V_x1
 *  				  - Edge 2 to vertex V_x2
 *  				  			...
 *  			...
 *
 *  		Vertex V_x			...
 *
 */

#ifndef SRC_GRAPH_GRAPH_H_
#define SRC_GRAPH_GRAPH_H_

#include <map>
#include <vector>
#include <cstdint>

namespace srcl_ctrl {

template<typename VertexNodeType>
class Vertex;

/****************************************************************************/
/*								 Edge  										*/
/****************************************************************************/
template<typename EdgeVertexType>
class Edge
{
public:
	Edge(EdgeVertexType *d = nullptr, double c = 0.0):
			dst_(d), cost_(c){};

	EdgeVertexType *dst_;
	double cost_;
};

/****************************************************************************/
/*								 Vertex										*/
/****************************************************************************/
template<typename VertexNodeType>
class Vertex
{
public:
	Vertex();
	Vertex(const VertexNodeType *node = nullptr):
		node_(node), vertex_id_(node->node_id_){};

	const VertexNodeType *node_;
	uint64_t vertex_id_;
	std::vector<Edge<Vertex<VertexNodeType>>> adj_;
};

/****************************************************************************/
/*								 Graph										*/
/****************************************************************************/
template<typename GraphNodeType>
class Graph
{
public:
	Graph(){};
	~Graph(){
		typename std::map<uint64_t, Vertex<GraphNodeType>*>::iterator it;
		for(it = vertex_map_.begin(); it != vertex_map_.end(); it++)
			delete it->second;
	};

private:
	std::map<uint64_t, Vertex<GraphNodeType>*> vertex_map_;

private:
	// This function checks if a vertex already exists in the graph.
	//	If yes, the functions returns the index of the existing vertex,
	//	otherwise it creates a new vertex.
	Vertex<GraphNodeType>* GetVertex(const GraphNodeType* vertex_node)
	{
		typename std::map<uint64_t, Vertex<GraphNodeType>*>::iterator it = vertex_map_.find((uint64_t)vertex_node->node_id_);

		if(it == vertex_map_.end())
		{
			Vertex<GraphNodeType>* new_vertex = new Vertex<GraphNodeType>(vertex_node);
			vertex_map_[vertex_node->node_id_] = new_vertex;
			return new_vertex;
		}

		return it->second;
	}

public:
	// This function is used to create a graph
	void AddEdge(GraphNodeType* src_node, GraphNodeType* dst_node, double cost)
	{
		Vertex<GraphNodeType>* src_vertex = GetVertex(src_node);
		Vertex<GraphNodeType>* dst_vertex = GetVertex(dst_node);

		Edge<Vertex<GraphNodeType>>* new_edge = new Edge<Vertex<GraphNodeType>>(dst_vertex,cost);
		src_vertex->adj_.push_back(*new_edge);
	};

	// This functions is used to access all vertices of a constructed graph
	std::vector<Vertex<GraphNodeType>*> GetGraphVertices()
	{
		typename std::map<uint64_t, Vertex<GraphNodeType>*>::iterator it;
		std::vector<Vertex<GraphNodeType>*> vertices;

		for(it = vertex_map_.begin(); it != vertex_map_.end(); it++)
		{
			vertices.push_back(it->second);
		}

		return vertices;
	};
};
}

#endif /* SRC_GRAPH_GRAPH_H_ */
