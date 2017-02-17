/*
 * test_squaregrid.cpp
 *
 *  Created on: Feb 22, 2016
 *      Author: rdu
 */

// standard libaray
#include <iostream>
#include <vector>
#include <ctime>

// opencv
#include "opencv2/opencv.hpp"

// user
#include "graph/graph.h"
#include "graph/astar.h"
#include "vis/graph_vis.h"
#include "vis/sgrid_vis.h"
#include "map/image_utils.h"
#include "geometry/graph_builder.h"
#include "geometry/sgrid_builder.h"

using namespace cv;
using namespace srcl_ctrl;

int main(int argc, char** argv )
{
	Mat input_map;
	bool use_input_image = false;

	Map_t<SquareGrid> sgrid_map;

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
			sgrid_map = SGridBuilderV2::BuildSquareGridMap(input_map, 16, 2);
			use_input_image = true;
		}
	}
	else{
		printf("Default test map is used \n");

		// create a empty grid
		std::shared_ptr<SquareGrid> grid = std::make_shared<SquareGrid>(12,12,95);

		// set occupancy for cells
		for(int i = 52; i <= 57; i++)
			grid->SetCellOccupancy(i, OccupancyType::OCCUPIED);

		for(int i = 88; i <= 93; i++)
			grid->SetCellOccupancy(i, OccupancyType::OCCUPIED);

		for(int i = 74; i <= 75; i++)
			grid->SetCellOccupancy(i, OccupancyType::OCCUPIED);

		for(int i = 0; i < 8; i++)
			grid->SetCellOccupancy(i,10, OccupancyType::OCCUPIED);

		for(int i = 24; i <= 28; i++)
			grid->SetCellOccupancy(i, OccupancyType::OCCUPIED);

		grid->SetCellOccupancy(58, OccupancyType::OCCUPIED);
		grid->SetCellOccupancy(87, OccupancyType::OCCUPIED);
		grid->SetCellOccupancy(22, OccupancyType::OCCUPIED);
		grid->SetCellOccupancy(34, OccupancyType::OCCUPIED);
		grid->SetCellOccupancy(46, OccupancyType::OCCUPIED);
		grid->SetCellOccupancy(118, OccupancyType::OCCUPIED);
		grid->SetCellOccupancy(119, OccupancyType::OCCUPIED);

		grid->SetCellOccupancy(7, OccupancyType::OCCUPIED);
		grid->SetCellOccupancy(19, OccupancyType::OCCUPIED);
		grid->SetCellOccupancy(31, OccupancyType::OCCUPIED);

		grid->SetCellOccupancy(66, OccupancyType::OCCUPIED);
		grid->SetCellOccupancy(81, OccupancyType::OCCUPIED);

		sgrid_map.data_model = grid;
	}

	/************************************************************************************/
	/* Below this point, a SquareGrid object should be available for graph construction */
	/************************************************************************************/

	/*** Construct a graph from the square grid ***/
	/*** the second argument determines if move along diagonal is allowed ***/
	std::shared_ptr<Graph<SquareCell*>> graph = GraphBuilder::BuildFromSquareGrid(sgrid_map.data_model,true);

	/*** Search path in the graph ***/
//	Vertex<SquareCell*> * start_vertex;
//	Vertex<SquareCell*> * finish_vertex;
//	if(use_input_image)
//	{
//		start_vertex = graph->GetVertexFromID(160);
//		finish_vertex = graph->GetVertexFromID(830);
//	}
//	else
//	{
//		start_vertex = graph->GetVertexFromID(0);
//		finish_vertex = graph->GetVertexFromID(143);
//	}
//
//	if(start_vertex == nullptr || finish_vertex == nullptr) {
//		std::cerr << "Invalid starting and finishing vertices, please choose two vertices in free space!" << std::endl;
//		std::cerr << "Use image \"example.png\" inside \\planning\\data folder for this demo." << std::endl;
//		return 0;
//	}
//
//	clock_t		exec_time;
//	exec_time = clock();
//	std::vector<Vertex<SquareCell*>*> path = AStar::Search(graph,start_vertex,finish_vertex);
//	exec_time = clock() - exec_time;
//	std::cout << "Searched in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;

	/*** Visualize the map and graph ***/
	Mat vis_img;

	/*** Image Layouts: (map) -> square grid -> graph -> path ***/
	/*** you can visualize the squre grid by itself or overlay it on the map image ***/
	if(sgrid_map.padded_image.empty())
		Vis::VisSquareGrid(*sgrid_map.data_model, vis_img);
	else
		Vis::VisSquareGrid(*sgrid_map.data_model, sgrid_map.padded_image, vis_img);

	/*** put the graph on top of the square grid ***/
	Vis::VisGraph(*graph, vis_img, vis_img, true);
	/*** put the path on top of the graph ***/
//	GraphVis::VisSquareGridPath(path, vis_img, vis_img);

	// display visualization result
	namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
	imshow("Processed Image", vis_img);

	waitKey(0);

	imwrite( "new_map_path_cmp2.jpg", vis_img);

	return 0;
}
