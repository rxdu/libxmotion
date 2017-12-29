
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "fastplot/image_plot.h"

using namespace cv;
using namespace librav;

int main()
{
    // Mat image;
    // image = imread("/home/rdu/Workspace/librav/data/intensity.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
    std::string file_name = "/home/rdu/Workspace/librav/data/intensity.jpg";

    Mat imagen = ImagePlot::ReadImageFile(file_name);
    ImagePlot::ShowImage(imagen, "test normal");

    Mat imagec = ImagePlot::ReadColorImage(file_name);
    ImagePlot::ShowImage(imagec, "test color");

    Mat imageg = ImagePlot::ReadGreyscaleImage(file_name);
    ImagePlot::ShowImage(imageg, "test greyscale");

    return 0;
}