/* 
 * octomap_server.h
 * 
 * Created on: May 26, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#ifndef OCTOMAP_SERVER_H
#define OCTOMAP_SERVER_H

#include <memory>
#include <cstdint>
#include <string>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/auto_drive.hpp"

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>

#include "map/librav_math.hpp"

namespace ivnav {

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

	std::shared_ptr<octomap::OcTree> octree_;
	TransMath::Transform3D octree_transf_;

public:
	void SetOctreeResolution(double new_res) { octree_res_ = new_res; };
	double GetOctreeResolution() { return octree_res_; };
	void SaveTreeToFile(std::string file_name);
	bool IsPositionOccupied(Position3Dd pos);

	void LcmTransformHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const librav_lcm_msgs::QuadrotorTransform* msg);
	void LcmLaserScanPointsHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const librav_lcm_msgs::LaserScanPoints_t* msg);

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
};

}

#endif /* OCTOMAP_SERVER_H */
