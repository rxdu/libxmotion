#include <iostream>

#include "cvdraw/cv_canvas.hpp"

using namespace librav;

int main()
{
    // Mat image;
    // image = imread("/home/rdu/Workspace/librav/data/intensity.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
    std::string file_name = "/home/rdu/Workspace/librav/data/intensity/intensity.jpg";
    cv::Mat image = cv::imread(file_name, CV_LOAD_IMAGE_COLOR);   // Read the file

    // CvDraw::ShowImage(file_name, "test greyscale");

    // CvCanvas canvas(600, 400);
    CvCanvas canvas(image);
    canvas.Show();

    return 0;
}