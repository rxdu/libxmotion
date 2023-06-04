/*
 * cv_io.cpp
 *
 * Created on: Mar 28, 2018 14:47
 * Description:
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "cvdraw/cv_io.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace xmotion;
using namespace cv;

cv::Mat CvIO::ReadImageFile(std::string img_file) { return imread(img_file); }

cv::Mat CvIO::ReadColorImage(std::string img_file) {
  return imread(img_file, cv::IMREAD_COLOR);
}

cv::Mat CvIO::ReadGrayscaleImage(std::string img_file) {
  return imread(img_file, cv::IMREAD_GRAYSCALE);
}

void CvIO::ShowImage(cv::Mat img, std::string window_name, bool save_img) {
  namedWindow(window_name, WINDOW_NORMAL);  // Create a window for display.
  imshow(window_name, img);                 // Show our image inside it.

  waitKey(0);  // Wait for a keystroke in the window

  if (save_img) imwrite(window_name + ".png", img);

  destroyWindow(window_name);
}

void CvIO::ShowImageFrame(cv::Mat img, std::string window_name,
                          int32_t frame_period_ms) {
  imshow(window_name, img);  // Show our image inside it.
  waitKey(frame_period_ms);  // Wait for a keystroke in the window
}

void CvIO::ShowImage(std::string file_name, std::string window_name) {
  cv::Mat img = imread(file_name);
  ShowImage(img, window_name, false);
}