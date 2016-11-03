/*
 * mission_utils.cpp
 *
 *  Created on: Nov 3, 2016
 *      Author: rdu
 */

#include <cmath>
#include <iostream>
#include <mission/mission_utils.h>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

using namespace srcl_ctrl;

std::vector<Position3Dd> MissionUtils::GetKeyTurningWaypoints(std::vector<Position3Dd>& wps)
{
	std::vector<Position3Dd> smoothed_points;

	// first remove undesired points at the connections between 2d/3d geomarks
	for(auto it = wps.begin(); it != wps.end() - 1; ++it)
	{
		Position3Dd pt1 = *it;
		Position3Dd pt2 = *(it+1);

		smoothed_points.push_back(pt1);

		// fix the connections between 2d and 3d vertices
		if(std::abs(pt1.z - pt2.z) > 0.05)
		{
			// ignore the last point if there is a 2d/3d connection
			//	between the last two points
			if(it + 2 != wps.end())
			{
				Position3Dd pt3 = *(it + 2);

				Eigen::Vector3d v1(pt2.x - pt1.x, pt2.y - pt1.y, pt2.z - pt1.z);
				Eigen::Vector3d v2(pt3.x - pt2.x, pt3.y - pt2.y, pt3.z - pt2.z);

				// skip next point if direction is opposite
				if(v1.dot(v2) <= 0)
					it++;
			}
		}
		else if(it + 2 == wps.end())
			smoothed_points.push_back(pt2);
	}

	// std::cout << "selected points: " << smoothed_points.size() << std::endl;

	// then remove intermediate points in a straight line
	std::vector<Position3Dd> minimum_points;

	if(smoothed_points.size() <= 2)
	{
		minimum_points = smoothed_points;
	}
	else
	{
		// add first waypoint
		minimum_points.push_back(smoothed_points.front());
		// check intermediate waypoints
		for(int cid = 1; cid < smoothed_points.size() - 1; cid++)
		{
			Position3Dd pt1 = smoothed_points[cid - 1];
			Position3Dd pt2 = smoothed_points[cid];
			Position3Dd pt3 = smoothed_points[cid + 1];

			Eigen::Vector3d v1 = Eigen::Vector3d(pt2.x - pt1.x, pt2.y - pt1.y, pt2.z - pt1.z).normalized();
			Eigen::Vector3d v2 = Eigen::Vector3d(pt3.x - pt2.x, pt3.y - pt2.y, pt3.z - pt2.z).normalized();
			Eigen::Vector3d e = v1 - v2;

			// |e| = sqrt[sin(theta)^2 + (1 - cos(theta))^2], |e| ~= 0.082 when theta = 5 degree
			if(e.norm() > 0.082)
			{
				minimum_points.push_back(smoothed_points[cid]);
			}
		}
		// add last waypoint
		minimum_points.push_back(smoothed_points.back());
	}

	return minimum_points;
}


