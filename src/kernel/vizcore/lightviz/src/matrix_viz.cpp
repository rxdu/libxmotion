/* 
 * matrix_viz.cpp
 * 
 * Created on: Mar 28, 2018 21:02
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "lightviz/matrix_viz.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "canvas/cv_draw.hpp"
#include "lightviz/details/matrix_draw.hpp"

using namespace librav;
using namespace cv;
using namespace CvDraw;

void LightViz::ShowMatrixAsImage(const Eigen::MatrixXd &matrix, std::string window_name, bool save_img)
{
    cv::Mat img;
    eigen2cv(matrix, img);
    img.convertTo(img, CV_8UC3);
    ShowImage(img, window_name, save_img);
}

void LightViz::ShowMatrixAsColorMap(const Eigen::MatrixXd &matrix, std::string window_name, bool save_img)
{
    cv::Mat color_img = CreateColorMapFromEigenMatrix(matrix);
    ShowImage(color_img, window_name, save_img);
}

void LightViz::ShowPathOnMatrixAsColorMap(const Eigen::MatrixXd &matrix, std::vector<RectGridIndex> waypoints, std::string window_name, bool save_img)
{
    cv::Mat color_img = CreateColorMapFromEigenMatrix(matrix);

    for (int i = 0; i < waypoints.size() - 1; ++i)
    {
        DrawLine(color_img, cv::Point(waypoints[i].GetX(), waypoints[i].GetY()),
                 cv::Point(waypoints[i + 1].GetX(), waypoints[i + 1].GetY()), cv::Scalar(244, 92, 66));
    }

    ShowImage(color_img, window_name, save_img);
}
