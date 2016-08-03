/*
 * test_rrtvis.cpp
 *
 *  Created on: Aug 3, 2016
 *      Author: rdu
 */

#include "vis/rrt_vis.h"

#include "map/map_info.h"
#include "planner/rrts_planner.h"

using namespace srcl_ctrl;
using namespace cv;

int main(int argc, char** argv)
{
	Mat vis_img;
	MapInfo map_info;

	RRTStarPlanner planner;
	std::vector<Position2Dd> path;

	planner.ConfigLocalPlanner();

	Position2Dd start,goal;
	start.x = 0;
	start.y = 0;
	goal.x = 1;
	goal.y = 1;
	planner.SetStartAndGoal(start, goal);
	planner.SearchSolution(start, goal, path);

	map_info.map_size_x = 1500;
	map_info.map_size_y = 1500;
	map_info.SetWorldSize(1.0, 1.0);

	RRTVis::CreateRRTCanvas(map_info.map_size_x,map_info.map_size_y, vis_img);
	RRTVis::VisRRTPath(path, map_info, vis_img, vis_img);

	// display visualization result
	namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
	imshow("Processed Image", vis_img);

	waitKey(0);
}


