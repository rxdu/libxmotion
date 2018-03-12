
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

    FastPlot::AddGraphLayer(Planner::BuildGraphFromSquareGrid(grid), img, img, false);
    FastPlot::ShowImage(img);

    return 0;
}