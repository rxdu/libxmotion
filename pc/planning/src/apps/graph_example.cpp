/*
 * example.cpp
 *
 *  Created on: Mar 27, 2016
 *      Author: rdu
 */

// standard libaray
#include <map2d/graph_builder.h>
#include <map2d/sgrid_builder.h>
#include <stdio.h>
#include <vector>
#include <ctime>
#include <tuple>

// opencv
#include "opencv2/opencv.hpp"

// user
#include "graph/graph.h"
#include "visualizer/graph_vis.h"

using namespace cv;
using namespace srcl_ctrl;

int main(int argc, char** argv )
{
	Mat input_image,map;
	bool use_input_image = false;

	std::shared_ptr<SquareGrid> grid;

	/*** check if user specifies an image ***/
	if ( argc == 2 )
	{
		input_image = imread( argv[1], IMREAD_GRAYSCALE );

		if (!input_image.data)
		{
			printf("No image data \n");
			return -1;
		}
		/*** create a square grid map from input image ***/
		else
		{
			/*** BuildSquareGridMap() returns both the square grid data structure    ***/
			/***  and the post-processed image, this image is usually not the same   ***/
			/***  with the original input image (after binarizing, padding). The     ***/
			/***  square grid or the graph can be visualized over this image without ***/
			/***  mis-placement. ***/
			std::tuple<std::shared_ptr<SquareGrid>, Mat> sg_map;

			sg_map = SGridBuilder::BuildSquareGridMap(input_image, 32);
			grid = std::get<0>(sg_map);
			map = std::get<1>(sg_map);

			/*** BuildSquareGrid() only returns the square grid data structure ***/
			//grid = SGridBuilder::BuildSquareGrid(input_image, 32);

			use_input_image = true;
		}
	}
	/*** otherwise, create a square grid map manually ***/
	else{
		// create a empty grid
		grid = std::make_shared<SquareGrid>(12,12,95);

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
	}

	/************************************************************************************/
	/* Below this point, a SquareGrid object should be available for graph construction */
	/************************************************************************************/

	/*** Construct a graph from the square grid ***/
	/*** the second argument determines if move along diagonal is allowed ***/
	std::shared_ptr<Graph<SquareCell>> graph = GraphBuilder::BuildFromSquareGrid(grid,true);

	/*** Search path in the graph ***/
	Vertex<SquareCell> * start_vertex;
	Vertex<SquareCell> * finish_vertex;
	if(use_input_image)
	{
		start_vertex = graph->GetVertexFromID(160);
		finish_vertex = graph->GetVertexFromID(830);
	}
	else
	{
		start_vertex = graph->GetVertexFromID(0);
		finish_vertex = graph->GetVertexFromID(143);
	}

	if(start_vertex == nullptr || finish_vertex == nullptr) {
		std::cerr << "Invalid starting and finishing vertices, please choose two vertices in free space!" << std::endl;
		std::cerr << "Use image \"example.png\" inside \\planning\\data folder for this demo." << std::endl;
		return 0;
	}

	clock_t		exec_time;
	exec_time = clock();
	std::vector<Vertex<SquareCell>*> path = graph->AStarSearch(start_vertex,finish_vertex);
	exec_time = clock() - exec_time;
	std::cout << "Searched in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;

	/*** Visualize the map and graph ***/
	GraphVis vis;
	Mat vis_img;

	/*** Image Layouts: (map) -> square grid -> graph -> path ***/
	/*** you can visualize the squre grid by itself or overlay it on the map image ***/
	if(map.empty())
		vis.VisSquareGrid(*grid, vis_img);
	else
		vis.VisSquareGrid(*grid, map, vis_img);

	/*** put the graph on top of the square grid ***/
	vis.VisSquareGridGraph(*graph, vis_img, vis_img, true);
	/*** put the path on top of the graph ***/
	vis.VisSquareGridPath(path, vis_img, vis_img);

	// display visualization result
	namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
	imshow("Processed Image", vis_img);

	waitKey(0);

	/*** uncomment this line if you want to save result into an image ***/
	// imwrite( "examples_result.jpg", vis_img);

	return 0;
}
