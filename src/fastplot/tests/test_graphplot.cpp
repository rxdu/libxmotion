
#include <memory>
#include <iostream>

#include "graph/graph.hpp"
#include "fastplot/fastplot.hpp"
#include "planner/graph_builder.hpp"

using namespace cv;
using namespace librav;

int main()
{
    std::shared_ptr<SquareGrid> grid = std::make_shared<SquareGrid>(3,3);

    grid->SetCellOccupancy(1,1, OccupancyType::OCCUPIED);
    grid->SetCellOccupancy(1,2, OccupancyType::OCCUPIED);

    // grid->SetCellOccupancy(2,2, OccupancyType::UNKONWN);

    cv::Mat img;
    FastPlot::CreateSquareGridLayer(grid, img);
    // FastPlot::PlotSquareGrid(grid);

    // std::shared_ptr<Graph_t<SquareCell*>> graph = std::make_shared<Graph_t<SquareCell*>>();

    // for(auto& cell_col : grid->grid_cells_)
    //     for(auto& cell : cell_col)
    //     {
    //         uint64_t current_nodeid = cell->id_;
    //         std::cout << "current id: " << current_nodeid << std::endl;

    //         if(cell->occupancy_ == OccupancyType::FREE) {
    //             auto neighbour_list = grid->GetNeighbours(current_nodeid,true);

    //             for(auto& neighbour : neighbour_list)
    //             {
    //                 if(neighbour->occupancy_ == OccupancyType::FREE)
    //                 {
    //                     double error_x,error_y, cost = 0;
    //                     error_x = std::abs(neighbour->position_.x - cell->position_.x);
    //                     error_y = std::abs(neighbour->position_.y - cell->position_.y);
    //                     cost = std::sqrt(error_x*error_x + error_y*error_y);

    //                     graph->AddEdge(cell, neighbour, cost);
    //                 }

    //                 std::cout << " - neighbour id: " << neighbour->id_ << std::endl;
    //             }
    //         }
    //     }

    // std::cout << "graph vertex num: " << graph->GetGraphVertices().size() << " , edge num: " << graph->GetGraphEdges().size() << std::endl;

    FastPlot::AddGraphLayer(Planner::BuildGraphFromSquareGrid(grid), img, img, false);
    FastPlot::ShowImage(img);

    return 0;
}