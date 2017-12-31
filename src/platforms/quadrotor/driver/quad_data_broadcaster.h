/*
 * quad_data_broadcaster.h
 *
 *  Created on: May 26, 2016
 *      Author: rdu
 */

#ifndef QUADROTOR_DRIVER_QUAD_DATA_BROADCASTER_H_
#define QUADROTOR_DRIVER_QUAD_DATA_BROADCASTER_H_

#include <memory>
#include <vector>

#include "eigen3/Eigen/Geometry"
#include <lcm/lcm-cpp.hpp>

#include "common/librav_types.hpp"
#include "control/quad_ctrl/state/quad_state.h"

namespace librav
{

class QuadDataBroadcaster
{
  public:
	QuadDataBroadcaster() = delete;
	QuadDataBroadcaster(std::shared_ptr<lcm::LCM> lcm_ptr);

	void SendQuadStateData(const QuadState &rs);
	void SendSystemTime(uint64_t sys_t);

  private:
	std::shared_ptr<lcm::LCM> lcm_;

	void SendQuadTransform(Point3f pos, Eigen::Quaterniond quat);
	void SendLaserPoints(const std::vector<Point3f> &pts);
	void SendLaserPoints(const std::vector<Point3f> &pts, Point3f pos, Eigen::Quaterniond quat);
};

}

#endif /* QUADROTOR_DRIVER_QUAD_DATA_BROADCASTER_H_ */
