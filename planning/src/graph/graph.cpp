/*
 * graph.cpp
 *
 *  Created on: Dec 9, 2015
 *      Author: rdu
 */

#include "graph.h"

using namespace srcl_ctrl;

Graph::Graph()
{

}

Graph::~Graph()
{
	std::map<uint64_t, Vertex*>::iterator it;
	for(it = vertex_map_.begin(); it != vertex_map_.end(); it++)
		delete it->second;
}

Vertex* Graph::GetVertex(const uint64_t vertex_id)
{
	std::map<uint64_t, Vertex*>::iterator it = vertex_map_.find(vertex_id);

	if(it == vertex_map_.end())
	{
		Vertex* new_vertex = new Vertex(vertex_id);
		vertex_map_[vertex_id] = new_vertex;

		return new_vertex;
	}

	return it->second;
}

void Graph::AddEdge(uint64_t src_id, uint64_t dst_id, double cost)
{
	Vertex* src_vertex = GetVertex(src_id);
	Vertex* dst_vertex = GetVertex(dst_id);

	src_vertex->adj_.push_back(Edge(dst_vertex,cost));
}


