/*
 * map_manager.cpp
 *
 *  Created on: Jan 28, 2016
 *      Author: rdu
 */

#include "map_manager.h"

using namespace srcl_ctrl;
using namespace cv;

MapManager::MapManager()
{

}

MapManager::~MapManager()
{

}

SquareGrid*  MapManager::CreateTestGridMap12N12Case1()
{
	// create a empty grid
	SquareGrid* grid = new SquareGrid(12,12,95);

	// set occupancy for cells
//	for(int i = 52; i <= 57; i++)
//		grid->SetCellOccupancy(i, OccupancyType::OCCUPIDED);
//
//	for(int i = 88; i <= 93; i++)
//		grid->SetCellOccupancy(i, OccupancyType::OCCUPIDED);

	for(int i = 0; i < 8; i++)
		grid->SetCellOccupancy(i,10, OccupancyType::OCCUPIED);

	grid->SetCellOccupancy(0,8, OccupancyType::INTERESTED);
	grid->SetCellOccupancy(0,9, OccupancyType::INTERESTED);
	grid->SetCellOccupancy(1,9, OccupancyType::INTERESTED);
	grid->SetCellOccupancy(6,9, OccupancyType::INTERESTED);
	grid->SetCellOccupancy(7,9, OccupancyType::INTERESTED);
	grid->SetCellOccupancy(2,2, OccupancyType::INTERESTED);
	grid->SetCellOccupancy(3,2, OccupancyType::INTERESTED);

	return grid;
}

SquareGrid*  MapManager::CreateTestGridMap12N12Case2()
{
	// create a empty grid
	SquareGrid* grid = new SquareGrid(12,12,95);

	// set occupancy for cells
	for(int i = 52; i <= 57; i++)
		grid->SetCellOccupancy(i, OccupancyType::OCCUPIED);

	for(int i = 88; i <= 93; i++)
		grid->SetCellOccupancy(i, OccupancyType::OCCUPIED);

	for(int i = 0; i < 8; i++)
		grid->SetCellOccupancy(i,10, OccupancyType::OCCUPIED);

	grid->SetCellOccupancy(0,8, OccupancyType::INTERESTED);
	grid->SetCellOccupancy(0,9, OccupancyType::INTERESTED);
	grid->SetCellOccupancy(1,9, OccupancyType::INTERESTED);
	grid->SetCellOccupancy(6,9, OccupancyType::INTERESTED);
	grid->SetCellOccupancy(7,9, OccupancyType::INTERESTED);
	grid->SetCellOccupancy(2,2, OccupancyType::INTERESTED);
	grid->SetCellOccupancy(3,2, OccupancyType::INTERESTED);

	return grid;
}

SquareGrid* MapManager::CreateTestGridMap25N25Case1()
{
	// create a empty grid
	SquareGrid* grid = new SquareGrid(25,25,95);

	// set occupancy for cells
	for(int i = 52; i <= 57; i++)
		grid->SetCellOccupancy(i, OccupancyType::OCCUPIED);

	for(int i = 88; i <= 93; i++)
		grid->SetCellOccupancy(i, OccupancyType::OCCUPIED);

	for(int i = 0; i < 8; i++)
		grid->SetCellOccupancy(i,10, OccupancyType::OCCUPIED);

	grid->SetCellOccupancy(0,8, OccupancyType::INTERESTED);
	grid->SetCellOccupancy(0,9, OccupancyType::INTERESTED);
	grid->SetCellOccupancy(1,9, OccupancyType::INTERESTED);
	grid->SetCellOccupancy(6,9, OccupancyType::INTERESTED);
	grid->SetCellOccupancy(7,9, OccupancyType::INTERESTED);
	grid->SetCellOccupancy(2,2, OccupancyType::INTERESTED);
	grid->SetCellOccupancy(3,2, OccupancyType::INTERESTED);

	return grid;
}

SquareGrid* MapManager::CreateTestGridMap12N12Astar()
{
	// create a empty grid
	SquareGrid* grid = new SquareGrid(12,12,95);

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

	return grid;
}

SquareGrid* MapManager::CreateTestGridMap3N3()
{
	// create a empty grid
	SquareGrid* grid = new SquareGrid(3,3,95);

	return grid;
}
