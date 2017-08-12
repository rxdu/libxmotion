/*
 * robot_state.h
 *
 *  Created on: Mar 1, 2016
 *      Author: rdu
 */

#ifndef NAVIGATION_ROBOT_STATE_H_
#define NAVIGATION_ROBOT_STATE_H_

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "common/control_types.h"
#include "control/quad_ctrl/data_types/quad_config.h"

namespace librav{

enum class QuadFlightType {
	X_TYPE,
	PLUS_TYPE,
};

struct QuadSensorData
{
	// sensor data
	unsigned char mono_image[IMG_RES_Y][IMG_RES_X];
	std::vector<Point3f> laser_points;
	IMUData imu_data;

	// data only available in simulator
	Point3f pos_i;
	Point3f vel_i;
	Point3f rot_i;
	Quaternion quat_i;
	Point3f rot_rate_b;
};

struct QuadCmd
{
	float ang_vel[4];
};

class QuadState {
public:
	QuadState();
	~QuadState() = default;

public:
	Point3f position_;
	Point3f velocity_;
	Point3f orientation_;
	Eigen::Quaterniond quat_;
	Point3f rotation_rate_;
	std::vector<Point3f> laser_points_;

	QuadFlightType quad_flight_type_;

public:
	double w_h_;
	const float g_;

	// quadrotor parameters
	double mass_;
	double arm_length_;
	const double kF_;
	const double kM_;

	double sim_step_;

public:
	void UpdateRobotState(const QuadSensorData& new_data);
};

}

#endif /* NAVIGATION_ROBOT_STATE_H_ */
