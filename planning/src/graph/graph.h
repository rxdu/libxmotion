/*
 * graph.h
 *
 *  Created on: Dec 9, 2015
 *      Author: rdu
 *
 *  Description:
 *  	1. This file defines template classes of the graph structure, for creating graphs
 *  	from trees and other similar data structures. The basic elements include:
 *  	 	a. Vertex
 *  	 	b. Edge
 *  	 	c. Graph
 *  	A vertex is associated with a node (such as a tree node); an edge connects two vertices;
 *  	A graph is a collection of vertices and the edges of each vertex.
 *
 *  	2. The graph uses unique IDs to index its vertices. So it's required that each
 *  	node used to construct the graph should have an unique ID with the name "node_id_".
 *
 *  	3. A visualized illustration of the graph structure
 *
 *  	Graph "G":
 *  		Vertex V1 - Edge E1_V1 to vertex V_x1
 *  				  - Edge E2_V1 to vertex V_x2
 *  				  			...
 *  		Vertex V2 - Edge E1_V2 to vertex V_x1
 *  				  - Edge E2_V2 to vertex V_x2
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
//	Vertex();
	Vertex(const VertexNodeType *node = nullptr):
		node_(node), vertex_id_(node->node_id_),
		is_checked_(false), is_under_checking_(false),
		search_parent_(nullptr),
		search_cost_so_far_(0){};

	const VertexNodeType *node_;
	uint64_t vertex_id_;
	std::vector<Edge<Vertex<VertexNodeType>>> adj_;

	// member variables for search
	bool is_checked_;
	bool is_under_checking_;
	double search_cost_so_far_;
	Vertex<VertexNodeType>* search_parent_;
	void ClearVertexSearchInfo(){
		search_cost_so_far_ = 0;
		is_checked_ = false;
		is_under_checking_ = false;
		search_parent_ = nullptr;
	}

	double GetEdgeCost(VertexNodeType dst_node)
	{
		double cost = -1;
		typename std::vector<Edge<Vertex<VertexNodeType>>>::iterator ite;
		for(ite = adj_.begin(); ite != adj_.end(); ite++)
		{
			if((*ite)->dst_.vertex_id_ == dst_node.vertex_id_)
			{
				cost = (*ite)->cost_;
				break;
			}
		}

		return cost;
	}
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

	// This function is used to reset the vertices for a new search
	void ResetGraphVertices()
	{
		typename std::map<uint64_t, Vertex<GraphNodeType>*>::iterator it;

		for(it = vertex_map_.begin(); it != vertex_map_.end(); it++)
		{
			it->second.ClearVertexSearchInfo();
		}
	};
};

}

#endif /* SRC_GRAPH_GRAPH_H_ */
