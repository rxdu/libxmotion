/*
 * graph.cpp
 *
 *  Created on: Dec 9, 2015
 *      Author: rdu
 */

#include <algorithm>
#include "graph.h"

using namespace srcl_ctrl;

//Graph::Graph()
//{
//
//}

//Graph::~Graph()
//{
//	std::map<uint64_t, Vertex*>::iterator it;
//	for(it = vertex_map_.begin(); it != vertex_map_.end(); it++)
//		delete it->second;
//}

//Vertex* Graph::GetVertex(const uint64_t vertex_id)
//{
//	std::map<uint64_t, Vertex*>::iterator it = vertex_map_.find(vertex_id);
//
//	if(it == vertex_map_.end())
//	{
//		Vertex* new_vertex = new Vertex(vertex_id);
//		vertex_map_[vertex_id] = new_vertex;
//
//		return new_vertex;
//	}
//
//	return it->second;
//}

//Vertex* Graph::GetVertex(const TreeNode* vertex_node)
//{
//	std::map<uint64_t, Vertex*>::iterator it = vertex_map_.find((uint64_t)vertex_node->node_id_);
//
//	if(it == vertex_map_.end())
//	{
//		Vertex* new_vertex = new Vertex((TreeNode*)vertex_node);
//		vertex_map_[vertex_node->node_id_] = new_vertex;
//		return new_vertex;
//	}
//
//	return it->second;
//}

//void Graph::AddEdge(uint64_t src_id, uint64_t dst_id, double cost)
//{
//	Vertex* src_vertex = GetVertex(src_id);
//	Vertex* dst_vertex = GetVertex(dst_id);
//
//	src_vertex->adj_.push_back(Edge(dst_vertex,cost));
//}
//
//void Graph::AddEdge(TreeNode* src_node, TreeNode* dst_node, double cost)
//{
//	Vertex* src_vertex = GetVertex(src_node);
//	Vertex* dst_vertex = GetVertex(dst_node);
//
//	src_vertex->adj_.push_back(Edge(dst_vertex,cost));
//}

//std::vector<Vertex*> Graph::GetGraphVertices()
//{
//	std::map<uint64_t, Vertex*>::iterator it;
//	std::vector<Vertex*> vertices;
//
//	for(it = vertex_map_.begin(); it != vertex_map_.end(); it++)
//	{
//		vertices.push_back(it->second);
//	}
//
//	return vertices;
//}
