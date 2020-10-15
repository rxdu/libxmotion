#include <iostream>
#include "image_io/image_io.hpp"
#include "cvdraw/cvdraw.hpp"

using namespace ivnav;

int main()
{
    std::string file = "/home/rdu/Pictures/accel.jpg";
    // std::string file = "/home/rdu/Pictures/ai_hw3_sim2.png";
    // std::string file = "/home/rdu/Pictures/test_small2_grey.png";
    // std::string file = "/home/rdu/Pictures/Lenna.jpg";

    MonoImageMatrix img = ImageIO::ReadImage(file);
    LightViz::ShowMatrixAsImage(img.data);
    LightViz::ShowMatrixAsColorMap(img.data);

    ImageIO::SaveToImage(img, "testjpg.jpg");
    ImageIO::SaveToImage(img, "testpng.png");
    ImageIO::SaveToImage(img, "testdefault");

    return 0;
}