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
#include "lcmtypes/srcl_ctrl.hpp"

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>

#include "local/transformation.h"

namespace srcl_ctrl {

typedef struct
{
	octomap::Pointcloud pc;
	octomath::Pose6D transf;
	octomap::point3d sensor_origin;
} PointCloudSnap;

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

	octomath::Pose6D base_pose_;
	octomath::Vector3 sensor_origin_;

	std::vector<PointCloudSnap> pc_snaps_;

	std::vector<octomap::Pointcloud> point_cloud_;
	octomap::Pointcloud current_pc_;
	octomap::ScanNode scan_node_;
	const int scan_buffer_size_;

public:
	std::shared_ptr<octomap::OcTree> octree_;
	utils::Transformation::Transform3D octree_transf_;

public:
	void SetOctreeResolution(double new_res) { octree_res_ = new_res; };
	double GetOctreeResolution() { return octree_res_; };
	void SaveTreeToFile(std::string file_name);
	bool IsPositionOccupied(Position3Dd pos);

	void LcmTransformHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_lcm_msgs::QuadrotorTransform* msg);
	void LcmLaserScanPointsHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_lcm_msgs::LaserScanPoints_t* msg);

protected:
	inline static void updateMinKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& min) {
		for (unsigned i = 0; i < 3; ++i)
			min[i] = std::min(in[i], min[i]);
	};

	inline static void updateMaxKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& max) {
		for (unsigned i = 0; i < 3; ++i)
			max[i] = std::max(in[i], max[i]);
	};

	void insertScan(const PointCloudSnap& pcs, std::shared_ptr<octomap::OcTree> m_octree);
};

}


#endif /* SRC_PLANNING_SRC_MAP_OCTOMAP_SERVER_H_ */
