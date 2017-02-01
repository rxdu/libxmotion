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

#include "graph/graph.h"
#include "geometry/square_grid/square_grid.h"
#include "nav_field.h"

namespace srcl_ctrl {

class ShortcutEval {
public:
	ShortcutEval(std::shared_ptr<SquareGrid> sgrid, std::shared_ptr<NavField<SquareCell*>> nav_field);

private:
	std::shared_ptr<SquareGrid> sgrid_;
	std::shared_ptr<NavField<SquareCell*>> nav_field_;

	double dist_weight;

	double EvaluateCellShortcutPotential(Vertex_t<SquareCell*>* eval_vtx);
	double CalcDirectDistance(Position2D start, Position2D goal, double cell_size, bool allow_diag);

public:
	void EvaluateGridShortcutPotential();
	Path_t<SquareCell*> SearchInNavField(Vertex_t<SquareCell*>* start_vtx, Vertex_t<SquareCell*>* goal_vtx);
};

}

#endif /* PLANNING_SRC_NAV_FIELD_SHORTCUT_EVAL_H_ */
