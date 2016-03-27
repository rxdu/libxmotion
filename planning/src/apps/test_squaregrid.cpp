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
#include "sgrid_builder.h"
#include "graph.h"
#include "graph_builder.h"
#include "graph_vis.h"
#include "image_utils.h"

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
			grid = SGridBuilder::BuildSquareGrid(input_map, 32);
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
	Graph<SquareCell>* graph = GraphBuilder::BuildFromSquareGrid(grid,true);

	// Search path in the graph


	// Visualize the map and graph
	GraphVis vis;
	Mat vis_result;

//	vis.DrawSquareGrid(grid, vis_result);
//	vis.DrawSquareGridGraph(graph, grid, vis_result);
	Mat bin_map, pad_map, vis_map;
	ImageUtils::BinarizeImage(input_map, bin_map, 200);
	ImageUtils::PadImageTo2Exp(bin_map, pad_map);
	vis.VisSquareGrid(grid, pad_map, vis_map);
	vis.VisSquareGridGraph(*graph, vis_map, vis_result, true);

	auto start_it = graph->GetVertexFromID(1710);
	auto finish_it = graph->GetVertexFromID(272);
//	auto start_it = graph->GetVertexFromID(0);
//	auto finish_it = graph->GetVertexFromID(1);

	clock_t		exec_time;

	exec_time = clock();
	std::vector<Vertex<SquareCell>*> path = graph->AStarSearch(start_it,finish_it);
	exec_time = clock() - exec_time;
	std::cout << "Searched in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;

	vis.VisSquareGridPath(path, vis_result, vis_result);

	namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
	imshow("Processed Image", vis_result);

	waitKey(0);

//	imwrite( "new_map_path_cmp2.jpg", vis_result);

	delete grid;
	delete graph;

	return 0;
}






