/*
 * test_template.cpp
 *
 *  Created on: April 14, 2016
 *      Author: rdu
 */


// standard libaray
#include <stdio.h>
#include <vector>
#include <ctime>
//#include <tuple>

// user
#include "graph.h"

using namespace srcl_ctrl;

int main(int argc, char** argv )
{
	std::vector<ExampleNode*> nodes;

	// create nodes
	for(int i = 0; i < 4; i++) {
		nodes.push_back(new ExampleNode(i));
	}

	// create a graph
	Graph<ExampleNode> graph;

	graph.AddEdge(*(nodes[0]), *(nodes[1]), 1.0);
	graph.AddEdge(*(nodes[0]), *(nodes[2]), 1.5);
	graph.AddEdge(*(nodes[1]), *(nodes[2]), 2.0);
	graph.AddEdge(*(nodes[2]), *(nodes[3]), 2.5);

	auto all_edges = graph.GetGraphEdges();

	for(auto e : all_edges)
		e.PrintEdge();

	for(auto e : nodes)
		delete e;

	return 0;
}



