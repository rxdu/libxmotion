/*
 * test_vis.cpp
 *
 *  Created on: Feb 17, 2017
 *      Author: rdu
 */

#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>
#include <highgui.h>

#include "vis/vis_utils.h"

using namespace std;
using namespace cv;
using namespace srcl_ctrl;

int main(int argc, char* argv[])
{
   auto width = 320;
   auto height = 320;
   Mat img = cv::Mat(cv::Size(width, height), CV_8UC3); // create background image

   auto center = cv::Point(width / 2, height / 2); // center point

   int lineType = 8;
   int thickness = 1;
   double tipLength = 0.1;

   img.setTo(0);                          // clear image - set to black

   for(long ag = 0; ag <= 360; ag += 36)
	   VisUtils::DrawArrow(img, center, 50, ag, Scalar(204,204,102));

   imshow("Arrowed Image", img);          // show image

   waitKey();

   return EXIT_SUCCESS;
}


