
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "fastplot/image_plot.hpp"

using namespace cv;
using namespace librav;

int main()
{
    // Mat image;
    // image = imread("/home/rdu/Workspace/librav/data/intensity.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
    std::string file_name = "/home/rdu/Workspace/librav/data/intensity.jpg";

    Mat imagen = FastPlot::ReadImageFile(file_name);
    FastPlot::ShowImage(imagen, "test normal");

    Mat imagec = FastPlot::ReadColorImage(file_name);
    FastPlot::ShowImage(imagec, "test color");

    Mat imageg = FastPlot::ReadGrayscaleImage(file_name);
    FastPlot::ShowImage(imageg, "test greyscale");

    return 0;
}