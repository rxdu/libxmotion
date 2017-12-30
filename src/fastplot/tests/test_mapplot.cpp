
#include <memory>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "fastplot/map_plot.hpp"
#include "fastplot/image_plot.hpp"

using namespace cv;
using namespace librav;

int main()
{
    std::shared_ptr<SquareGrid> grid = std::make_shared<SquareGrid>(3,3);

    grid->SetCellOccupancy(1,1, OccupancyType::OCCUPIED);
    grid->SetCellOccupancy(2,2, OccupancyType::UNKONWN);

    // cv::Mat img;
    // MapPlot::PlotSquareGrid(grid, img);
    // ImagePlot::ShowImage(img, "test square grid vis");
    FastPlot::PlotSquareGrid(grid);

    return 0;
}