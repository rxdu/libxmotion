/*
 * graph.h
 *
 *  Created on: Dec 9, 2015
 *      Author: rdu
 */

#ifndef SRC_GRAPH_GRAPH_H_
#define SRC_GRAPH_GRAPH_H_

#include <map>
#include <vector>
#include <cstdint>
#include "graph_types.h"

namespace srcl_ctrl {

template<class GraphNodeType>
class Graph
{
public:
	Graph(){};
	~Graph(){
		std::map<uint64_t, Vertex<GraphNodeType>*>::iterator it;
		for(it = vertex_map_.begin(); it != vertex_map_.end(); it++)
			delete it->second;
	};

private:
	std::map<uint64_t, Vertex<GraphNodeType>*> vertex_map_;

private:
	Vertex<GraphNodeType>* GetVertex(const GraphNodeType* vertex_node)
	{
		std::map<uint64_t, Vertex<GraphNodeType>*>::iterator it = vertex_map_.find((uint64_t)vertex_node->node_id_);

		if(it == vertex_map_.end())
		{
			Vertex<GraphNodeType>* new_vertex = new Vertex<GraphNodeType>();
			new_vertex->vertex_id_ = vertex_node->vertex_id_;
			vertex_map_[vertex_node->node_id_] = new_vertex;
			return new_vertex;
		}

		return it->second;
	}

public:
	void AddEdge(GraphNodeType* src_node, GraphNodeType* dst_node, double cost)
	{
		Vertex<GraphNodeType>* src_vertex = GetVertex(src_node);
		Vertex<GraphNodeType>* dst_vertex = GetVertex(dst_node);

		src_vertex->adj_.push_back(Edge<GraphNodeType>(dst_vertex,cost));
	};

	std::vector<Vertex<GraphNodeType>*> GetGraphVertices()
	{
		std::map<uint64_t, Vertex<GraphNodeType>*>::iterator it;
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
