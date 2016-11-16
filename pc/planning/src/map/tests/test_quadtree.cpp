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
#include "graph/astar.h"
#include "vis/graph_vis.h"

#include "map/map_type.h"
#include "geometry/graph_builder.h"
#include "map/image_utils.h"
#include "geometry/qtree_builder.h"

using namespace cv;
using namespace srcl_ctrl;

int main(int argc, char** argv )
{
	Map_t<QuadTree> qtree_map;
	Mat image_raw,image_disp;

    if ( argc != 2 )
    {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }

    image_raw = imread( argv[1], IMREAD_GRAYSCALE );

    if ( !image_raw.data )
    {
        printf("No image data \n");
        return -1;
    }

    // example to use quadtree builder
    qtree_map = QTreeBuilder::BuildQuadTreeMap(image_raw, 6);

//    uint32_t test_id = qtree_map.data_model->GetIDFromPosition(0, 128);
//    std::cout << "map size: " << qtree_map.padded_image.cols << "," << qtree_map.padded_image.rows << std::endl;
//    std::cout << "tested id: " << test_id << std::endl;

    Mat image_tree, image_nodes;
    Mat vis_map;
    GraphVis::VisQuadTree(*qtree_map.data_model, qtree_map.padded_image, image_tree, TreeVisType::ALL_SPACE);
//    TreeNode* node = tree->leaf_nodes_.at(0);
//    vis.DrawQTreeSingleNode(node, image_tree, image_nodes);
    std::vector<QuadTreeNode*> free_leaves;
    std::vector<QuadTreeNode*>::iterator it;
    for(it = qtree_map.data_model->leaf_nodes_.begin(); it != qtree_map.data_model->leaf_nodes_.end(); it++)
    {
    	if((*it)->occupancy_ == OccupancyType::FREE)
    		free_leaves.push_back((*it));
    }
    GraphVis::VisQTreeNodes(free_leaves, image_tree, image_nodes);

//    Mat image_dummy;
//    vis.DrawQTreeWithDummies(tree,builder.padded_img_, image_dummy);

    // build a graph from quadtree
	std::shared_ptr<Graph<QuadTreeNode*>> graph = GraphBuilder::BuildFromQuadTree(qtree_map.data_model);
	Mat image_graph;
	GraphVis::VisQTreeGraph(*graph, image_tree, image_graph, true,false);

	// try a* search
	std::vector<Vertex<QuadTreeNode*>*> vertices = graph->GetGraphVertices();
	std::cout<<"vertex number: "<<vertices.size()<<std::endl;

	Vertex<QuadTreeNode*>* start_vertex;
	Vertex<QuadTreeNode*>* end_vertex;
	std::vector<Vertex<QuadTreeNode*>*> traj;

	start_vertex = vertices[0];
	end_vertex = vertices[15];
//	start_vertex = graph->GetVertexFromID(172);
//	end_vertex = graph->GetVertexFromID(20);

	clock_t		exec_time;

	std::cout<<"Start from "<< start_vertex->vertex_id_<<" and finish at "<< end_vertex->vertex_id_<< std::endl;
	exec_time = clock();
	traj = AStar::Search(graph, start_vertex, end_vertex);
	exec_time = clock() - exec_time;
	std::cout << "Searched in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;

	Mat path_img;
	GraphVis::VisQTreeGraphPath(traj, image_graph, path_img);

	image_disp = path_img;

//    imwrite( "new_map_path_cmp1.jpg", image_disp);

    namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
    imshow("Processed Image", image_disp);

    waitKey(0);

    return 0;
}
