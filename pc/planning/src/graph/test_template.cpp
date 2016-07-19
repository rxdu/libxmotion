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
#include "graph/graph.h"
#include "graph/bds_example.h"

using namespace srcl_ctrl;

struct LiftedBDSExample: public BDSBase<LiftedBDSExample>
{
	LiftedBDSExample(uint64_t id):
		BDSBase<LiftedBDSExample>(id){};
	~LiftedBDSExample(){};

	std::vector<BDSExample*> history;

	double GetHeuristic(const LiftedBDSExample& other_struct) const {
		return 0.0;
	}
};

int main(int argc, char** argv )
{
	std::vector<BDSExample*> nodes;

	// create nodes
	for(int i = 0; i < 9; i++) {
		nodes.push_back(new BDSExample(i));
	}

	// create a graph
	Graph_t<BDSExample> graph;

	std::cout << "BDSExample node type: " << typeid(*(nodes[0])).name() << std::endl;

	graph.AddEdge(*(nodes[0]), *(nodes[1]), 1.0);
	graph.AddEdge(*(nodes[0]), *(nodes[3]), 1.5);
	graph.AddEdge(*(nodes[1]), *(nodes[0]), 2.0);
	graph.AddEdge(*(nodes[1]), *(nodes[4]), 2.5);
	graph.AddEdge(*(nodes[1]), *(nodes[2]), 1.0);
	graph.AddEdge(*(nodes[2]), *(nodes[1]), 1.5);
	graph.AddEdge(*(nodes[2]), *(nodes[5]), 2.0);
	graph.AddEdge(*(nodes[3]), *(nodes[0]), 2.5);
	graph.AddEdge(*(nodes[3]), *(nodes[4]), 2.5);
	graph.AddEdge(*(nodes[4]), *(nodes[1]), 2.5);
	graph.AddEdge(*(nodes[4]), *(nodes[3]), 2.5);
	graph.AddEdge(*(nodes[4]), *(nodes[5]), 2.5);
	graph.AddEdge(*(nodes[5]), *(nodes[2]), 2.5);
	graph.AddEdge(*(nodes[5]), *(nodes[4]), 2.5);
	graph.AddEdge(*(nodes[5]), *(nodes[8]), 2.5);
	graph.AddEdge(*(nodes[7]), *(nodes[4]), 2.5);
	graph.AddEdge(*(nodes[7]), *(nodes[8]), 2.5);
	graph.AddEdge(*(nodes[8]), *(nodes[5]), 2.5);
	graph.AddEdge(*(nodes[8]), *(nodes[7]), 2.5);

	std::cout << "----------------------------------- graph1 ----------------------------------- " << std::endl;
	std::vector<Edge_t<BDSExample>> all_edges = graph.GetGraphEdges();
	for(auto& e : all_edges) {
		e.PrintEdge();
	}

	Graph_t<BDSExample*> graph2;

	graph2.AddEdge((nodes[0]), (nodes[1]), 1.0);
	graph2.AddEdge((nodes[0]), (nodes[3]), 1.5);
	graph2.AddEdge((nodes[1]), (nodes[0]), 2.0);
	graph2.AddEdge((nodes[1]), (nodes[4]), 2.5);
	graph2.AddEdge((nodes[1]), (nodes[2]), 1.0);
	graph2.AddEdge((nodes[2]), (nodes[1]), 1.5);
	graph2.AddEdge((nodes[2]), (nodes[5]), 2.0);
	graph2.AddEdge((nodes[3]), (nodes[0]), 2.5);
	graph2.AddEdge((nodes[3]), (nodes[4]), 2.5);
	graph2.AddEdge((nodes[4]), (nodes[1]), 2.5);
	graph2.AddEdge((nodes[4]), (nodes[3]), 2.5);
	graph2.AddEdge((nodes[4]), (nodes[5]), 2.5);
	graph2.AddEdge((nodes[5]), (nodes[2]), 2.5);
	graph2.AddEdge((nodes[5]), (nodes[4]), 2.5);
	graph2.AddEdge((nodes[5]), (nodes[8]), 2.5);
	graph2.AddEdge((nodes[7]), (nodes[4]), 2.5);
	graph2.AddEdge((nodes[7]), (nodes[8]), 2.5);
	graph2.AddEdge((nodes[8]), (nodes[5]), 2.5);
	graph2.AddEdge((nodes[8]), (nodes[7]), 2.5);

	std::cout << "----------------------------------- graph2 ----------------------------------- " << std::endl;
	auto all_edges2 = graph2.GetGraphEdges();
	for(auto& e : all_edges2) {
		e.PrintEdge();
	}

	Graph_t<const BDSExample&> graph3;

	graph3.AddEdge(*(nodes[0]), *(nodes[1]), 1.0);
	graph3.AddEdge(*(nodes[0]), *(nodes[3]), 1.5);
	graph3.AddEdge(*(nodes[1]), *(nodes[0]), 2.0);
	graph3.AddEdge(*(nodes[1]), *(nodes[4]), 2.5);
	graph3.AddEdge(*(nodes[1]), *(nodes[2]), 1.0);
	graph3.AddEdge(*(nodes[2]), *(nodes[1]), 1.5);
	graph3.AddEdge(*(nodes[2]), *(nodes[5]), 2.0);
	graph3.AddEdge(*(nodes[3]), *(nodes[0]), 2.5);
	graph3.AddEdge(*(nodes[3]), *(nodes[4]), 2.5);
	graph3.AddEdge(*(nodes[4]), *(nodes[1]), 2.5);
	graph3.AddEdge(*(nodes[4]), *(nodes[3]), 2.5);
	graph3.AddEdge(*(nodes[4]), *(nodes[5]), 2.5);
	graph3.AddEdge(*(nodes[5]), *(nodes[2]), 2.5);
	graph3.AddEdge(*(nodes[5]), *(nodes[4]), 2.5);
	graph3.AddEdge(*(nodes[5]), *(nodes[8]), 2.5);
	graph3.AddEdge(*(nodes[7]), *(nodes[4]), 2.5);
	graph3.AddEdge(*(nodes[7]), *(nodes[8]), 2.5);
	graph3.AddEdge(*(nodes[8]), *(nodes[5]), 2.5);
	graph3.AddEdge(*(nodes[8]), *(nodes[7]), 2.5);

	std::cout << "----------------------------------- graph3 ----------------------------------- " << std::endl;
	auto all_edges3 = graph3.GetGraphEdges();
	for(auto& e : all_edges3) {
		e.PrintEdge();
	}

	Graph_t<LiftedBDSExample> graph4;
	LiftedBDSExample node1(0), node2(1),node3(2);
	std::vector<BDSExample*> g4v1,g4v2,g4v3;
	g4v1.push_back(nodes[0]);
	g4v1.push_back(nodes[2]);
	g4v1.push_back(nodes[1]);
	node1.history = g4v1;

	g4v2.push_back(nodes[2]);
	g4v2.push_back(nodes[1]);
	g4v2.push_back(nodes[3]);
	node2.history = g4v2;

	g4v3.push_back(nodes[3]);
	g4v3.push_back(nodes[5]);
	g4v3.push_back(nodes[7]);
	node3.history = g4v3;

	graph4.AddEdge(node1, node2, 1.0);
	graph4.AddEdge(node1, node3, 2.0);
	graph4.AddEdge(node2, node3, 2.5);

	std::cout << "----------------------------------- graph4 ----------------------------------- " << std::endl;
	auto all_edges4 = graph4.GetGraphEdges();
	for(auto& e : all_edges4) {
		e.PrintEdge();
	}

	std::cout << "\n----------------------------------- A* on graph1 ----------------------------------- " << std::endl;
	std::cout << "test a* of graph1" << std::endl;
	auto path = graph.AStarSearch(graph.GetVertexFromID(0), graph.GetVertexFromID(8));

	std::cout << "\n----------------------------------- A* on graph2 ----------------------------------- " << std::endl;
	std::cout << "test a* of graph2" << std::endl;
	auto path2 = graph2.AStarSearch(graph2.GetVertexFromID(0), graph2.GetVertexFromID(8));

	std::cout << "\n----------------------------------- A* on graph2 ----------------------------------- " << std::endl;
	std::cout << "test a* of graph3" << std::endl;
	auto v1 = graph3.GetVertexFromID(0);
	auto v2 = graph3.GetVertexFromID(8);
	std::cout << "id1: " << v1->bundled_data_.GetID() << " id2: " << v2->bundled_data_.GetID() << std::endl;
	auto path3 = graph3.AStarSearch(graph3.GetVertexFromID(0), graph3.GetVertexFromID(8));

	std::cout << "\n----------------------------------- A* on graph4 ----------------------------------- " << std::endl;
	std::cout << "test a* of graph4" << std::endl;
	auto path4 = graph4.AStarSearch(graph4.GetVertexFromID(0), graph4.GetVertexFromID(2));

//	for(auto& e : path)
//		std::cout << "id: " << e->vertex_id_ << std::endl;

	// delete all nodes
	for(auto e : nodes)
		delete e;

	return 0;
}



