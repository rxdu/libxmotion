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

#include <vertex.h>
#include <astar.h>

namespace srcl_ctrl {

/****************************************************************************/
/*							   Graph Node						  			*/
/****************************************************************************/
// an example of node that can be associated with a vertex, this node can be
//	either a "struct" or a "class", only need to provide the node_id_ attribute
struct ExampleNode{
	ExampleNode(uint64_t id):node_id_(id){}

	const uint64_t node_id_;

	// you can add more attributes here
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
		// graph is only responsible to recycle of memory for vertices, the node which
		//	each vertex is associated to needs to be recycled by the quadtree/square_grid
		//	structure
		for(auto it = vertex_map_.begin(); it != vertex_map_.end(); it++)
			delete it->second;
	};

private:
	std::map<uint64_t, Vertex<GraphNodeType>*> vertex_map_;
	AStar<Vertex<GraphNodeType>> astar_;

private:
	// This function checks if a vertex already exists in the graph.
	//	If yes, the functions returns the index of the existing vertex,
	//	otherwise it creates a new vertex.
	Vertex<GraphNodeType>* GetVertex(GraphNodeType* vertex_node)
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

	// This function is used to reset the vertices for a new search
	void ResetGraphVertices()
	{
		typename std::map<uint64_t, Vertex<GraphNodeType>*>::iterator it;

		for(it = vertex_map_.begin(); it != vertex_map_.end(); it++)
		{
			it->second->ClearVertexSearchInfo();
		}
	};

public:
	// This function is used to create a graph
	void AddEdge(GraphNodeType* src_node, GraphNodeType* dst_node, double cost)
	{
		Vertex<GraphNodeType>* src_vertex = GetVertex(src_node);
		Vertex<GraphNodeType>* dst_vertex = GetVertex(dst_node);

//		Edge<Vertex<GraphNodeType>>* new_edge = new Edge<Vertex<GraphNodeType>>(dst_vertex,cost);
		Edge<Vertex<GraphNodeType>> new_edge(src_vertex, dst_vertex,cost);
		src_vertex->edges_.push_back(new_edge);
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

	// This functions is used to access all edges of a constructed graph
	std::vector<Edge<Vertex<GraphNodeType>>> GetGraphEdges()
	{
		typename std::map<uint64_t, Vertex<GraphNodeType>*>::iterator it;
		std::vector<Edge<Vertex<GraphNodeType>>> edges;

		for(it = vertex_map_.begin(); it != vertex_map_.end(); it++)
		{
			Vertex<GraphNodeType>* vertex = it->second;
			for(auto ite = vertex->edges_.begin(); ite != vertex->edges_.end(); ite++) {
				auto itedge = std::find(edges.begin(), edges.end(), (*ite));

				if(itedge == edges.end())
					edges.push_back(*ite);
			}
		}

		return edges;
	};

	// This function return the vertex with specified id
	Vertex<GraphNodeType>* GetVertexFromID(uint64_t vertex_id)
	{
		typename std::map<uint64_t, Vertex<GraphNodeType>*>::iterator it = vertex_map_.find(vertex_id);

		return (*it).second;
	}

	// Perform A* Search and return a path represented by a serious of vertices
	std::vector<Vertex<GraphNodeType>*> AStarSearch(Vertex<GraphNodeType> *start, Vertex<GraphNodeType> *goal)
	{
		// clear previous search information before new search
		ResetGraphVertices();

		// do a* search and return search result
		return astar_.Search(start, goal);
	}
};

}

#endif /* SRC_GRAPH_GRAPH_H_ */
