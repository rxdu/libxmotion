/*
 * sgrid_vis.h
 *
 *  Created on: Feb 9, 2017
 *      Author: rdu
 */

#ifndef PLANNING_SRC_VIS_SGRID_VIS_H_
#define PLANNING_SRC_VIS_SGRID_VIS_H_

#include "opencv2/opencv.hpp"

#include "graph/graph.h"
#include "geometry/square_grid/square_grid.h"
#include "nav_field/nav_field.h"

namespace srcl_ctrl {

namespace Vis
{
	// square grid visualization
	void VisSquareGrid(const SquareGrid& grid, cv::OutputArray _dst);
	void VisAbstractSquareGrid(const SquareGrid& grid, cv::OutputArray _dst);
	void VisSquareGrid(const SquareGrid& grid, cv::InputArray _src, cv::OutputArray _dst);

	void VisSquareGridNavField(const SquareGrid& grid, const NavField<SquareCell*>& nav_field, Vertex_t<SquareCell*>* start_vtx, cv::InputArray _src, cv::OutputArray _dst, bool show_id);
	void VisSquareGridLocalNavField(const SquareGrid& grid, const NavField<SquareCell*>& nav_field, Vertex_t<SquareCell*>* center_vtx, cv::InputArray _src, cv::OutputArray _dst, uint16_t sensor_range);
	void VisSquareGridShortcutPotential(const NavField<SquareCell*>& nav_field, cv::InputArray _src, cv::OutputArray _dst);
};

}



#endif /* PLANNING_SRC_VIS_SGRID_VIS_H_ */
