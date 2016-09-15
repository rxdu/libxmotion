/*
 * quad_flattraj.h
 *
 *  Created on: Aug 29, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_POLYOPT_QUAD_FLATTRAJ_H_
#define PLANNING_SRC_POLYOPT_QUAD_FLATTRAJ_H_

#include <cstdint>

#include "lcmtypes/comm.hpp"

#include "polyopt/polytraj_curve.h"

namespace srcl_ctrl {

typedef struct {
	double x;
	double y;
	double z;
	double yaw;
}QuadFlatOutput;

typedef struct {
	PolyTrajCurve seg_x;
	PolyTrajCurve seg_y;
	PolyTrajCurve seg_z;
	PolyTrajCurve seg_yaw;

	double t_start;
	double t_end;

	void print()
	{
		seg_x.print();
		seg_y.print();
		seg_z.print();
		seg_yaw.print();
	}
}QuadFlatOutputSeg;

class QuadFlatTraj {
public:
	QuadFlatTraj();
	~QuadFlatTraj();

public:
	std::vector<QuadFlatOutputSeg> traj_segs_;

public:
	void AddTrajSeg(const std::vector<std::vector<double>>& seg_coeffs, double ts, double te);

	QuadFlatOutput GetTrajPointPos(double t);

	srcl_msgs::PolynomialCurve_t GenerateNonDimPolyCurveLCMMsg()
	{
		srcl_msgs::PolynomialCurve_t msg;

		msg.seg_num = traj_segs_.size();
		uint32_t seg_idx = 0;
		for(auto& seg : traj_segs_)
		{
			srcl_msgs::PolyCurveSegment_t seg_msg;

			seg_msg.coeffs_x = seg.seg_x.param_.coeffs;
			seg_msg.coeffs_y = seg.seg_y.param_.coeffs;
			seg_msg.coeffs_z = seg.seg_z.param_.coeffs;

			seg_msg.t_start = 0;
			seg_msg.t_end = 1.0;

			msg.segments.push_back(seg_msg);
		}

		return msg;
	}

	void clear() { traj_segs_.clear(); };
	void print();
};

}


#endif /* PLANNING_SRC_POLYOPT_QUAD_FLATTRAJ_H_ */
