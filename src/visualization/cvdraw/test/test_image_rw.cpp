#include <iostream>

#include "cvdraw/cvdraw.hpp"

using namespace xmotion;

int main()
{
    // Mat image;
    // image = imread("/home/rdu/Workspace/librav/data/intensity.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
    std::string file_name = "/home/rdu/Pictures/3dr_solo_sim.png";

    CvIO::ShowImage(file_name, "test greyscale");

    return 0;
}