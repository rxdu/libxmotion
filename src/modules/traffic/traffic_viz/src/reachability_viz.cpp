/* 
 * reachability_viz.cpp
 * 
 * Created on: Oct 30, 2018 09:13
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "traffic_viz/reachability_viz.hpp"
#include "decomp/square_grid_viz.hpp"

using namespace ivnav;

void LightViz::ShowTStateSpace(TStateSpace &space, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    SquareGrid grid(space.GetSSize(), space.GetVSize());

    // auto cells = space.GetAllStateCells();

    // for (auto row : cells)
    //     for (auto cell : row)
    //     {
    //         // remap cells to a square grid
    //         int32_t s_idx = cell->id / space.GetVSize();
    //         int32_t v_idx = cell->id % space.GetVSize();
    //         // std::cout << "viz idx: " << s_idx << " , " << v_idx << std::endl;

    //         // grid.GetCell(cell->id)->cost_map = cell->count / 100.0;
    //         double cost = 0;
    //         int32_t stat = 0;
    //         // TODO fix this
    //         // for (int i = 0; i < cell->occupancy_stats.size(); ++i)
    //         //     stat += cell->occupancy_stats[i];
    //         cost = static_cast<double>(stat) / (space.sim_number_per_ctrl_ * space.control_set_size_);
    //         // if (cost != 0)
    //         //     std::cout << cost << "+";
    //         grid.GetCell(s_idx, space.GetVSize() - v_idx - 1)->cost_map = cost;
    //     }
    // std::cout << std::endl;

    // ShowSquareGridGraphCostOnly(&grid, pixel_per_unit, window_name, save_img);
    // ShowSquareGridGraphCost(&grid, pixel_per_unit, window_name, save_img);
    ShowSquareGrid(&grid, pixel_per_unit, window_name, save_img);
}