/*
 * quad_flattraj.h
 *
 *  Created on: Aug 29, 2016
 *      Author: rdu
 */

#ifndef COMMON_QUAD_FLATTRAJ_H_
#define COMMON_QUAD_FLATTRAJ_H_

#include <cstdint>

#include "lcmtypes/librav.hpp"

#include "common/poly_curve.h"

namespace librav {

typedef struct {
	double x;
	double y;
	double z;
	double yaw;
}QuadFlatOutput;

typedef struct {
	PolyCurve seg_x;
	PolyCurve seg_y;
	PolyCurve seg_z;
	PolyCurve seg_yaw;

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
	QuadFlatTraj()=default;
	~QuadFlatTraj()=default;

public:
	std::vector<QuadFlatOutputSeg> traj_segs_;

public:
	void AddTrajSeg(const std::vector<std::vector<double>>& seg_coeffs, double ts, double te);
	QuadFlatOutput GetTrajPointPos(double t);

	void clear() { traj_segs_.clear(); traj_segs_.resize(0); };
	void print();
};

}

#endif /* COMMON_QUAD_FLATTRAJ_H_ */
