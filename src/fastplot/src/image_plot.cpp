/* 
 * image_plot.cpp
 * 
 * Created on: Nov 27, 2017 16:47
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#include "fastplot/image_plot.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace librav;
using namespace cv;

cv::Mat ImagePlot::ReadImageFile(std::string img_file)
{
    return imread(img_file);
}

cv::Mat ImagePlot::ReadColorImage(std::string img_file)
{
    return imread(img_file, CV_LOAD_IMAGE_COLOR);
}
    
cv::Mat ImagePlot::ReadGreyscaleImage(std::string img_file)
{
    return imread(img_file, CV_LOAD_IMAGE_GRAYSCALE);
}

void ImagePlot::ShowImage(cv::Mat img, std::string window_name)
{
    namedWindow( window_name, WINDOW_NORMAL );    // Create a window for display.
    imshow( window_name, img );                     // Show our image inside it.

    waitKey(0);                                     // Wait for a keystroke in the window

    destroyWindow( window_name );
}