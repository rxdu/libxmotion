/*
 * octomap_server.h
 *
 *  Created on: May 26, 2016
 *      Author: rdu
 */

#ifndef SRC_PLANNING_SRC_MAP_OCTOMAP_SERVER_H_
#define SRC_PLANNING_SRC_MAP_OCTOMAP_SERVER_H_

#include <memory>
#include <cstdint>

#include <lcm/lcm-cpp.hpp>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>

#include "lcmtypes/comm.hpp"

namespace srcl_ctrl {

class OctomapServer {
public:
	OctomapServer(std::shared_ptr<lcm::LCM> lcm);
	~OctomapServer();

private:
	std::shared_ptr<lcm::LCM> lcm_;

	double octree_res_;
	bool save_tree_;
	uint64_t loop_count_;

public:
	void LcmLaserScanPointsHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_msgs::LaserScanPoints_t* msg);
	void SaveTreeToFile();
};

}


#endif /* SRC_PLANNING_SRC_MAP_OCTOMAP_SERVER_H_ */
