/* 
 * dense_grid_test.cpp
 * 
 * Created on: Oct 10, 2018 06:16
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#include <stdio.h>
#include <vector>

#include "gtest/gtest.h"

#include "graph/graph.hpp"

using namespace robotnav;

struct DenseGridTest : testing::Test
{
	DenseGridTest()
	{
		for (int i = 0; i < 9; i++)
			nodes.push_back(new TestState(i));
	}

	virtual ~DenseGridTest()
	{
		for (auto &nd : nodes)
			delete nd;
	}

	std::vector<TestState *> nodes;
};

TEST_F(DenseGridTest, VertexMod)
{
	// create a graph
	// Graph<TestState *> graph;

	// ASSERT_EQ(graph.GetTotalVertexNumber(), 0) << "Graph should have no vertex now";

	// graph.AddVertex(nodes[0]);
	// graph.AddVertex(nodes[1]);

	// ASSERT_EQ(graph.GetTotalVertexNumber(), 2) << "Failed to add vertices to pointer-type graph ";

	// ASSERT_EQ(graph.FindVertex(0)->vertex_id_, 0) << "Failed to find added vertex by associated state ID from pointer-type graph ";
	// ASSERT_EQ(graph.FindVertex(nodes[1])->vertex_id_, 1) << "Failed to find added vertex by associated state from pointer-type graph ";

	// graph.RemoveVertex(0);

	// ASSERT_EQ(graph.GetTotalVertexNumber(), 1) << "Failed to remove vertex by associated state ID from pointer-type graph ";
	// ASSERT_TRUE(graph.FindVertex(0) == graph.vertex_end()) << "Failed to remove vertex by associated state ID from pointer-type graph ";
	// ASSERT_EQ(graph.FindVertex(1)->vertex_id_, 1) << "Removed wrong vertex by associated state ID from pointer-type graph ";

	// graph.RemoveVertex(nodes[1]);

	// ASSERT_EQ(graph.GetTotalVertexNumber(), 0) << "Failed to remove vertex by associated state from pointer-type graph ";
	// ASSERT_TRUE(graph.FindVertex(1) == graph.vertex_end()) << "Failed to remove vertex by associated state from pointer-type graph ";
}
