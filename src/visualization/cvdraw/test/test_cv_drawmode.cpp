#include <iostream>

#include "cvdraw/cvdraw.hpp"

using namespace xmotion;

int main() {
  // Mat image;
  // image = imread("/home/rdu/Workspace/librav/data/intensity.jpg",
  // cv::IMREAD_COLOR);   // Read the file
  std::string file_name = "/home/rdu/Pictures/3dr_solo_sim.png";
  cv::Mat image = cv::imread(file_name, cv::IMREAD_COLOR);  // Read the file

  // CvDraw::ShowImage(file_name, "test greyscale");

  CvCanvas canvas(600, 400);

  // raster mode
  {
    canvas.DrawPoint({300, 200}, 5);
    canvas.DrawCircle({300, 200}, 35, CvColors::yellow_color, 3);
    canvas.DrawLine({50, 100}, {550, 100});
    canvas.DrawLine({80, 50}, {520, 150}, CvColors::green_color, 2);
    canvas.DrawLine({80, 150}, {520, 50}, CvColors::orange_color, 2);
    canvas.DrawArrowedLine({300, 150}, {300, 50}, 0.3, CvColors::red_color, 3);

    canvas.DrawRectangle({20, 20}, {580, 380}, CvColors::purple_color, 2);

    std::vector<CPoint> polyline{{80, 200}, {300, 350}, {520, 200}};
    canvas.DrawPolyline(polyline, false, CvColors::palegreen_color, 5);
    std::vector<CPoint> cpolyline{{150, 200}, {300, 350}, {450, 200}};
    canvas.DrawPolyline(cpolyline, true, CvColors::silver_color, 5);

    canvas.DrawEllipse({300, 200}, cv::Size(150.0, 50.0), 135, 0, 360);
    canvas.DrawEllipse({300, 200}, cv::Size(150.0, 50.0), 45, 0, 360);

    std::vector<CPoint> fpolyline{{100, 250}, {50, 350}, {150, 350}};
    canvas.FillPoly(fpolyline, CvColors::navy_color);

    std::vector<CPoint> fpolyline2{{500, 250}, {450, 350}, {550, 350}};
    canvas.FillConvexPoly(fpolyline2, CvColors::navy_color);

    canvas.WriteText("demo0.5", {20, 200}, 0.5);
    canvas.WriteText("demo1.0", {50, 180}, 1.0, CvColors::cyan_color);
    canvas.WriteText("demo1.2", {400, 180}, 1.2);
  }

  canvas.Show();
  canvas.SavePaint("draw_demo");

  // vector mode
  canvas.Clear();
  canvas.SetMode(CvCanvas::DrawMode::Vector);

  {
    canvas.DrawPoint({300 / 600.0, 200 / 400.0}, 5);
    canvas.DrawCircle({300 / 600.0, 200 / 400.0}, 35, CvColors::yellow_color,
                      3);
    canvas.DrawLine({50 / 600.0, 100 / 400.0}, {550 / 600.0, 100 / 400.0});
    canvas.DrawLine({80 / 600.0, 50 / 400.0}, {520 / 600.0, 150 / 400.0},
                    CvColors::green_color, 2);
    canvas.DrawLine({80 / 600.0, 150 / 400.0}, {520 / 600.0, 50 / 400.0},
                    CvColors::orange_color, 2);
    canvas.DrawArrowedLine({300 / 600.0, 150 / 400.0},
                           {300 / 600.0, 50 / 400.0}, 0.3, CvColors::red_color,
                           3);

    canvas.DrawRectangle({20 / 600.0, 20 / 400.0}, {580 / 600.0, 380 / 400.0},
                         CvColors::purple_color, 2);

    std::vector<CPoint> polyline{{80 / 600.0, 200 / 400.0},
                                 {300 / 600.0, 350 / 400.0},
                                 {520 / 600.0, 200 / 400.0}};
    canvas.DrawPolyline(polyline, false, CvColors::palegreen_color, 5);
    std::vector<CPoint> cpolyline{{150 / 600.0, 200 / 400.0},
                                  {300 / 600.0, 350 / 400.0},
                                  {450 / 600.0, 200 / 400.0}};
    canvas.DrawPolyline(cpolyline, true, CvColors::silver_color, 5);

    canvas.DrawEllipse({300 / 600.0, 200 / 400.0}, cv::Size(150.0, 50.0), 135,
                       0, 360);
    canvas.DrawEllipse({300 / 600.0, 200 / 400.0}, cv::Size(150.0, 50.0), 45, 0,
                       360);

    std::vector<CPoint> fpolyline{{100 / 600.0, 250 / 400.0},
                                  {50 / 600.0, 350 / 400.0},
                                  {150 / 600.0, 350 / 400.0}};
    canvas.FillPoly(fpolyline, CvColors::navy_color);

    std::vector<CPoint> fpolyline2{{500 / 600.0, 250 / 400.0},
                                   {450 / 600.0, 350 / 400.0},
                                   {550 / 600.0, 350 / 400.0}};
    canvas.FillConvexPoly(fpolyline2, CvColors::navy_color);

    canvas.WriteText("demo0.5", {20 / 600.0, 200 / 400.0}, 0.5);
    canvas.WriteText("demo1.0", {50 / 600.0, 180 / 400.0}, 1.0,
                     CvColors::cyan_color);
    canvas.WriteText("demo1.2", {400 / 600.0, 180 / 400.0}, 1.2);
  }

  canvas.Show();

  CvCanvas canvas2(10);
  canvas2.Resize(0, 60, 0, 40);

  {
    canvas2.DrawPoint({30.0, 20.0}, 5);
    canvas2.DrawCircle({30.0, 20.0}, 35, CvColors::yellow_color, 3);
    canvas2.DrawLine({5.0, 10.0}, {55.0, 10.0});
    canvas2.DrawLine({8.0, 5.0}, {52.0, 15.0}, CvColors::green_color, 2);
    canvas2.DrawLine({8.0, 15.0}, {52.0, 5.0}, CvColors::orange_color, 2);
    canvas2.DrawArrowedLine({30.0, 15.0}, {30.0, 5.0}, 0.3, CvColors::red_color,
                            3);

    canvas2.DrawRectangle({2.0, 2.0}, {58.0, 38.0}, CvColors::purple_color, 2);

    std::vector<CPoint> polyline{{8.0, 20.0}, {30.0, 35.0}, {52.0, 20.0}};
    canvas2.DrawPolyline(polyline, false, CvColors::palegreen_color, 5);
    std::vector<CPoint> cpolyline{{15.0, 20.0}, {30.0, 35.0}, {45.0, 20.0}};
    canvas2.DrawPolyline(cpolyline, true, CvColors::silver_color, 5);

    canvas2.DrawEllipse({30.0, 20.0}, cv::Size(150.0, 50.0), 135, 0, 360);
    canvas2.DrawEllipse({30.0, 20.0}, cv::Size(150.0, 50.0), 45, 0, 360);

    std::vector<CPoint> fpolyline{{10.0, 25.0}, {5.0, 35.0}, {15.0, 35.0}};
    canvas2.FillPoly(fpolyline, CvColors::navy_color);

    std::vector<CPoint> fpolyline2{{50.0, 25.0}, {45.0, 35.0}, {55.0, 35.0}};
    canvas2.FillConvexPoly(fpolyline2, CvColors::navy_color);

    canvas2.WriteText("demo0.5", {2.0, 20.0}, 0.5);
    canvas2.WriteText("demo1.0", {5.0, 18.0}, 1.0, CvColors::cyan_color);
    canvas2.WriteText("demo1.2", {40.0, 18.0}, 1.2);
  }

  canvas2.Show();

  return 0;
}