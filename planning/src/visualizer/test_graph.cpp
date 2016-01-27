/*
 * test_graph.cpp
 *
 *  Created on: Dec 15, 2015
 *      Author: rdu
 */

// standard libaray
#include <stdio.h>
#include <vector>

// opencv
#include "opencv2/opencv.hpp"

// quad_tree
#include "qtree_builder.h"
#include "graph_builder.h"
#include "astar.h"
#include "graph_vis.h"

using namespace cv;
using namespace srcl_ctrl;

int main(int argc, char** argv )
{
    if ( argc != 2 )
    {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }

    Mat image_raw;
    image_raw = imread( argv[1], IMREAD_GRAYSCALE );

    if ( !image_raw.data )
    {
        printf("No image data \n");
        return -1;
    }

    // example to use quadtree builder
    QTreeBuilder builder;
    QuadTree* tree = builder.BuildQuadTree(image_raw, 6);

    Mat image_tree, image_nodes;
    GraphVis vis;
    vis.DrawQuadTree(tree, builder.padded_img_, image_tree, TreeVisType::ALL_SPACE);
//    TreeNode* node = tree->leaf_nodes_.at(0);
//    vis.DrawQTreeSingleNode(node, image_tree, image_nodes);
    std::vector<QuadTreeNode*> free_leaves;
    std::vector<QuadTreeNode*>::iterator it;
    for(it = tree->leaf_nodes_.begin(); it != tree->leaf_nodes_.end(); it++)
    {
    	if((*it)->occupancy_ == OccupancyType::FREE)
    		free_leaves.push_back((*it));
    }
    vis.DrawQTreeNodes(free_leaves, image_tree, image_nodes);

    Mat image_dummy;
    vis.DrawQTreeWithDummies(tree,builder.padded_img_, image_dummy);

    // build a graph from quadtree
    Graph<QuadTreeNode>* graph;

	graph = GraphBuilder::BuildFromQuadTree(tree);
	Mat image_graph;
	vis.DrawQTreeGraph(graph, tree, image_tree, image_graph);

	AStar<Graph<QuadTreeNode>> astarsearch;

	std::vector<Vertex<QuadTreeNode>*> vertices = graph->GetGraphVertices();
//	astarsearch.Search(vertices[0], vertices[50]);

//    imwrite( "free_graph.jpg", image_graph );

    namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
    imshow("Processed Image", image_graph);

    waitKey(0);

    return 0;
}



