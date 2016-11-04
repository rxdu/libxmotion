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
#include <string>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/comm.hpp"

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>

#include "utils/src/frame/transformation.h"

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
	std::string save_tree_name_;

	Position3Dd pos_;
	Eigen::Quaterniond quat_;
	utils::Transformation::Transform3D transf_;
	octomath::Pose6D base_pose_;
	octomath::Vector3 sensor_origin_;

	std::vector<octomap::Pointcloud> point_cloud_;
	octomap::Pointcloud current_pc_;
	octomap::ScanNode scan_node_;
	const int scan_buffer_size_;

public:
	std::shared_ptr<octomap::OcTree> octree_;

public:
	void SetOctreeResolution(double new_res) { octree_res_ = new_res; };
	double GetOctreeResolution() { return octree_res_; };
	void SaveTreeToFile(std::string file_name);

	void LcmTransformHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_lcm_msgs::QuadrotorTransform* msg);
	void LcmLaserScanPointsHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_lcm_msgs::LaserScanPoints_t* msg);
};

}


#endif /* SRC_PLANNING_SRC_MAP_OCTOMAP_SERVER_H_ */
