/*
 * test_squaregrid.cpp
 *
 *  Created on: Feb 22, 2016
 *      Author: rdu
 */

// standard libaray
#include <stdio.h>
#include <vector>
#include <ctime>

// opencv
#include "opencv2/opencv.hpp"

// user
#include "map_manager.h"
#include "qtree_builder.h"
#include "graph_builder.h"
#include "astar.h"
#include "graph_vis.h"

using namespace cv;
using namespace srcl_ctrl;

int main(int argc, char** argv )
{
	Mat input_map;
	MapManager map_manager;
	SquareGrid* grid;

	if ( argc == 2 )
	{
		input_map = imread( argv[1], IMREAD_GRAYSCALE );

		if (!input_map.data)
		{
			printf("No image data \n");
			return -1;
		}
		else
		{
			// TODO
			// The map manager should be able to read a image
			//	and generate a grid map from the input image
		}
	}
	else{
		printf("Default test map is used \n");

		// Use map manager to create a test map
		grid = map_manager.CreateTestGridMap12N12Astar();
		//    	grid = map_manager.CreateTestGridMap3N3();
	}

	/************************************************************************************/
	/* Below this point, a SquareGrid object should be available for graph construction */
	/************************************************************************************/

	// Construct a graph from the grid map
	Graph<SquareCell>* graph = GraphBuilder::BuildFromQuadTree(grid);

	// Search path in the graph


	// Visualize the map and graph
	GraphVis vis;
	Mat vis_result;

//	vis.DrawSquareGrid(grid, vis_result);
	vis.DrawSquareGridGraph(graph, grid, vis_result);

	auto start_it = graph->GetVertexFromID(132);
	auto finish_it = graph->GetVertexFromID(13);

	clock_t		exec_time;

	exec_time = clock();
	std::vector<Vertex<SquareCell>*> path = graph->AStarSearch(start_it,finish_it);
	exec_time = clock() - exec_time;
	std::cout << "Searched in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;

	vis.DrawSquareGridPath(graph, grid, path, vis_result);

	namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
	imshow("Processed Image", vis_result);

	waitKey(0);

	//    imwrite( "astar.jpg", vis_result);

	return 0;
}






