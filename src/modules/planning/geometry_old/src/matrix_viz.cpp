/* 
 * matrix_viz.cpp
 * 
 * Created on: Mar 28, 2018 21:02
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "geometry/matrix_viz.hpp"

#include "cvdraw/cvdraw.hpp"
#include "geometry/matrix_draw.hpp"

#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace robotnav;
using namespace cv;

void LightViz::ShowMatrixAsImage(const Eigen::MatrixXd &matrix, std::string window_name, bool save_img)
{
    cv::Mat img;
    eigen2cv(matrix, img);
    img.convertTo(img, CV_8UC3);
    CvIO::ShowImage(img, window_name, save_img);
}

void LightViz::ShowMatrixAsColorMap(const Eigen::MatrixXd &matrix, std::string window_name, bool save_img)
{
    cv::Mat color_img = MatrixViz::CreateColorMapFromEigenMatrix(matrix);
    CvIO::ShowImage(color_img, window_name, save_img);
}

void LightViz::ShowPathOnMatrixAsColorMap(const Eigen::MatrixXd &matrix, std::vector<RectGridIndex> waypoints, std::string window_name, bool save_img)
{
    cv::Mat color_img = MatrixViz::CreateColorMapFromEigenMatrix(matrix);

    CvCanvas canvas(color_img);

    for (int i = 0; i < waypoints.size() - 1; ++i)
    {
        canvas.DrawLine({static_cast<double>(waypoints[i].GetX()), static_cast<double>(waypoints[i].GetY())},
                        {static_cast<double>(waypoints[i + 1].GetX()), static_cast<double>(waypoints[i + 1].GetY())}, 
                        cv::Scalar(244, 92, 66));
    }

    CvIO::ShowImage(color_img, window_name, save_img);
}
