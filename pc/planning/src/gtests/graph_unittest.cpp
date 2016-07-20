/*
 * graph_unittest.cpp
 *
 *  Created on: Jul 20, 2016
 *      Author: rdu
 */

#include <stdio.h>
#include <vector>

#include "gtest/gtest.h"

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

struct GraphTemplateTest: testing::Test
{
	std::vector<BDSExample*> nodes;

	GraphTemplateTest()
	{
		for(int i = 0; i < 9; i++) {
			nodes.push_back(new BDSExample(i));
		}
	}

	virtual ~GraphTemplateTest()
	{
		for(auto& nd : nodes)
			delete nd;
	}
};

TEST_F(GraphTemplateTest, ValueType)
{
	// create a graph
	Graph_t<BDSExample> graph;

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

	ASSERT_NE(graph.GetGraphVertices().size(), 0) << "Failed to add a vertex of value type to the graph";

	auto path = graph.AStarSearch(graph.GetVertexFromID(0), graph.GetVertexFromID(8));
	ASSERT_NE(path.size(), 0) << "Failed to find a path of value type with A* while there exists one";
}

TEST_F(GraphTemplateTest, PointerType)
{
	Graph_t<BDSExample*> graph;

	graph.AddEdge((nodes[0]), (nodes[1]), 1.0);
	graph.AddEdge((nodes[0]), (nodes[3]), 1.5);
	graph.AddEdge((nodes[1]), (nodes[0]), 2.0);
	graph.AddEdge((nodes[1]), (nodes[4]), 2.5);
	graph.AddEdge((nodes[1]), (nodes[2]), 1.0);
	graph.AddEdge((nodes[2]), (nodes[1]), 1.5);
	graph.AddEdge((nodes[2]), (nodes[5]), 2.0);
	graph.AddEdge((nodes[3]), (nodes[0]), 2.5);
	graph.AddEdge((nodes[3]), (nodes[4]), 2.5);
	graph.AddEdge((nodes[4]), (nodes[1]), 2.5);
	graph.AddEdge((nodes[4]), (nodes[3]), 2.5);
	graph.AddEdge((nodes[4]), (nodes[5]), 2.5);
	graph.AddEdge((nodes[5]), (nodes[2]), 2.5);
	graph.AddEdge((nodes[5]), (nodes[4]), 2.5);
	graph.AddEdge((nodes[5]), (nodes[8]), 2.5);
	graph.AddEdge((nodes[7]), (nodes[4]), 2.5);
	graph.AddEdge((nodes[7]), (nodes[8]), 2.5);
	graph.AddEdge((nodes[8]), (nodes[5]), 2.5);
	graph.AddEdge((nodes[8]), (nodes[7]), 2.5);

	ASSERT_NE(graph.GetGraphVertices().size(), 0) << "Failed to add a vertex of pointer type to the graph";

	auto path = graph.AStarSearch(graph.GetVertexFromID(0), graph.GetVertexFromID(8));
	ASSERT_NE(path.size(), 0) << "Failed to find a path of pointer type with A* while there exists one";
}

TEST_F(GraphTemplateTest, ConstRefType)
{
	Graph_t<const BDSExample&> graph;

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

	ASSERT_NE(graph.GetGraphVertices().size(), 0) << "Failed to add a vertex of const reference type to the graph";

	auto path = graph.AStarSearch(graph.GetVertexFromID(0), graph.GetVertexFromID(8));
	ASSERT_NE(path.size(), 0) << "Failed to find a path of const reference type with A* while there exists one";
}

TEST_F(GraphTemplateTest, LiftedGraphTest)
{
	Graph_t<LiftedBDSExample> graph;

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

	graph.AddEdge(node1, node2, 1.0);
	graph.AddEdge(node1, node3, 2.0);
	graph.AddEdge(node2, node3, 2.5);

	ASSERT_NE(graph.GetGraphVertices().size(), 0) << "Failed to add a lifted vertex to the graph";

	auto path = graph.AStarSearch(graph.GetVertexFromID(0), graph.GetVertexFromID(2));
	ASSERT_NE(path.size(), 0) << "Failed to find a path in the lifted graph with A* while there exists one";
}
