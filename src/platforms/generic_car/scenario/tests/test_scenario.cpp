#include "scenario/left_right_turn.hpp"
#include "fastplot/fastplot.hpp"

using namespace librav;

int main()
{
    auto grid = LeftRightTurnScenario::CreateSqaureGrid();

    cv::Mat img;
    FastPlot::CreateSquareGridLayer(grid, img);

    // FastPlot::AddGraphLayer(Planner::BuildGraphFromSquareGrid(grid), img, img, false);
    FastPlot::ShowImage(img);

    return 0;
}