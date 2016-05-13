/*
 * test_quadtree.cpp
 *
 *  Created on: Dec 15, 2015
 *      Author: rdu
 */

// standard libaray
#include <stdio.h>
#include <vector>
#include <ctime>
#include <tuple>

// opencv
#include "opencv2/opencv.hpp"

// quad_tree
#include "qtree_builder.h"
#include "graph_builder.h"
#include "astar.h"
#include "graph_vis.h"
#include "image_utils.h"

using namespace cv;
using namespace srcl_ctrl;

int main(int argc, char** argv )
{
    if ( argc != 2 )
    {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }

    Mat map;
    Mat image_raw,image_disp;

    image_raw = imread( argv[1], IMREAD_GRAYSCALE );

    if ( !image_raw.data )
    {
        printf("No image data \n");
        return -1;
    }

    // example to use quadtree builder
    std::shared_ptr<QuadTree> tree;

    std::tuple<std::shared_ptr<QuadTree>, Mat> qt_map;
    qt_map = QTreeBuilder::BuildQuadTreeMap(image_raw, 6);
    tree = std::get<0>(qt_map);
    map = std::get<1>(qt_map);

    Mat image_tree, image_nodes;
    GraphVis vis;
    Mat vis_map;
    vis.VisQuadTree(*tree, map, image_tree, TreeVisType::ALL_SPACE);
//    TreeNode* node = tree->leaf_nodes_.at(0);
//    vis.DrawQTreeSingleNode(node, image_tree, image_nodes);
    std::vector<QuadTreeNode*> free_leaves;
    std::vector<QuadTreeNode*>::iterator it;
    for(it = tree->leaf_nodes_.begin(); it != tree->leaf_nodes_.end(); it++)
    {
    	if((*it)->occupancy_ == OccupancyType::FREE)
    		free_leaves.push_back((*it));
    }
    vis.VisQTreeNodes(free_leaves, image_tree, image_nodes);

//    Mat image_dummy;
//    vis.DrawQTreeWithDummies(tree,builder.padded_img_, image_dummy);

    // build a graph from quadtree
	std::shared_ptr<Graph<QuadTreeNode>> graph = GraphBuilder::BuildFromQuadTree(tree);
	Mat image_graph;
	vis.VisQTreeGraph(*graph, image_tree, image_graph, true,false);

	// try a* search
	std::vector<Vertex<QuadTreeNode>*> vertices = graph->GetGraphVertices();
	std::cout<<"vertex number: "<<vertices.size()<<std::endl;

	Vertex<QuadTreeNode>* start_vertex;
	Vertex<QuadTreeNode>* end_vertex;
	std::vector<Vertex<QuadTreeNode>*> traj;

	start_vertex = vertices[0];
	end_vertex = vertices[15];
//	start_vertex = graph->GetVertexFromID(172);
//	end_vertex = graph->GetVertexFromID(20);

	clock_t		exec_time;

	std::cout<<"Start from "<< start_vertex->vertex_id_<<" and finish at "<< end_vertex->vertex_id_<< std::endl;
	exec_time = clock();
	traj = graph->AStarSearch(start_vertex, end_vertex);
	exec_time = clock() - exec_time;
	std::cout << "Searched in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;

	Mat path_img;
	vis.VisQTreeGraphPath(traj, image_graph, path_img);

	image_disp = path_img;

//    imwrite( "new_map_path_cmp1.jpg", image_disp);

    namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
    imshow("Processed Image", image_disp);

    waitKey(0);

    return 0;
}



