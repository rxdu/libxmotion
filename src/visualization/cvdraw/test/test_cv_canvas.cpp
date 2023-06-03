#include <iostream>

#include "cvdraw/cv_canvas.hpp"

using namespace xmotion;

int main() {
  // Mat image;
  // image = imread("/home/rdu/Workspace/librav/data/intensity.jpg",
  // cv::IMREAD_COLOR);   // Read the file
  std::string file_name =
      "/home/rdu/Workspace/librav/data/intensity/intensity.jpg";
  cv::Mat image = cv::imread(file_name, cv::IMREAD_COLOR);  // Read the file

  // CvDraw::ShowImage(file_name, "test greyscale");

  CvCanvas canvas(600, 400);
  canvas.DrawPoint({300, 200}, 5);
  canvas.DrawCircle({300, 200}, 35, CvColors::yellow_color, 3);
  canvas.Show();

  canvas.Clear();
  canvas.Show();

  canvas.Resize(300, 200);
  canvas.Show();

  // canvas.SavePaint("test_canvas");

  return 0;
}