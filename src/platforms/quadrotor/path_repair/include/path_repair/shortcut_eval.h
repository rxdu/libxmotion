/*
 * shortcut_eval.h
 *
 *  Created on: Jan 31, 2017
 *      Author: rdu
 */

#ifndef PLANNING_SRC_NAV_FIELD_SHORTCUT_EVAL_H_
#define PLANNING_SRC_NAV_FIELD_SHORTCUT_EVAL_H_

#include <memory>
#include <cstdint>

#include "planning/graph/graph.h"
#include "legacy/square_grid.h"
#include "path_repair/nav_field.h"

namespace librav {

class ShortcutEval {
public:
	ShortcutEval(std::shared_ptr<SquareGrid> sgrid, std::shared_ptr<NavField<SquareCell*>> nav_field);

private:
	std::shared_ptr<SquareGrid> sgrid_;
	std::shared_ptr<NavField<SquareCell*>> nav_field_;

	double EvaluateCellShortcutPotential(Vertex_t<SquareCell*>* eval_vtx, uint16_t sensor_range);
	double CalcDirectDistance(Position2Di start, Position2Di goal, double cell_size, bool allow_diag);

public:
	double dist_weight_;

public:
	void EvaluateGridShortcutPotential(uint16_t sensor_range);
	Path_t<SquareCell*> SearchInNavField(Vertex_t<SquareCell*>* start_vtx, Vertex_t<SquareCell*>* goal_vtx);
};

}

#endif /* PLANNING_SRC_NAV_FIELD_SHORTCUT_EVAL_H_ */
