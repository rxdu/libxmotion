/*
 * octomap_server.cpp
 *
 *  Created on: May 26, 2016
 *      Author: rdu
 */

#include <iostream>
#include <sstream>

#include "local3d/octomap_server.h"

using namespace srcl_ctrl;

OctomapServer::OctomapServer(std::shared_ptr<lcm::LCM> lcm):
		lcm_(lcm),
		octree_(new octomap::OcTree(0.01))
{
	lcm_->subscribe("vis_data_laser_scan_points",&OctomapServer::LcmLaserScanPointsHandler, this);
}

OctomapServer::~OctomapServer()
{
	delete octree_;
}

void OctomapServer::SaveTreeToFile()
{
	octree_->writeBinary("test_tree.bt");
}

void OctomapServer::LcmLaserScanPointsHandler(
		const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const srcl_msgs::LaserScanPoints_t* msg)
{
	std::cout << "points received " << std::endl;

	octomap::OcTree octree(0.01);

	for(auto& pt : msg->points)
	{
		octomap::point3d origin, end;

		origin.x() = 0;
		origin.y() = 0;
		origin.z() = 0;

		end.x() = pt.x;
		end.y() = pt.y;
		end.z() = pt.z;

		octree.insertRay(origin, end);
	}

	srcl_msgs::Octomap_t octomap_msg;

	octomap_msg.binary = true;
	octomap_msg.resolution = 0.01;
	octomap_msg.id = octree.getTreeType();

	octree.prune();
	std::stringstream datastream;

	if (octree.writeBinaryData(datastream))
	{
		std::string datastring = datastream.str();
		octomap_msg.data = std::vector<int8_t>(datastring.begin(), datastring.end());
		octomap_msg.data_size = octomap_msg.data.size();
	}

	lcm_->publish("hummingbird_laser_octomap", &octomap_msg);
}
