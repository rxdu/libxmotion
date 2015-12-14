/*
 * graph_builder.cpp
 *
 *  Created on: Dec 14, 2015
 *      Author: rdu
 */

#include "graph_builder.h"

using namespace srcl_ctrl;

GraphBuilder::GraphBuilder():
		graph_(nullptr)
{

}

GraphBuilder::~GraphBuilder()
{

}

void GraphBuilder::BuildGraphFromQuadTree(QuadTree *tree)
{
	graph_ = new Graph();

	tree->cell_res_;
}


