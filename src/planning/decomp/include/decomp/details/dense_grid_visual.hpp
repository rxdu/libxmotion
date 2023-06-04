/*
 * dense_grid_visual.hpp
 *
 * Created on: Mar 28, 2018 21:02
 * Description:
 *
 * Reference:
 * [1]
 * https://www.learnopencv.com/applycolormap-for-pseudocoloring-in-opencv-c-python/
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef DENSE_GRID_VISUAL_HPP
#define DENSE_GRID_VISUAL_HPP

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "decomp/dense_grid.hpp"
#include "cvdraw/cvdraw.hpp"

namespace xmotion {
cv::Mat CreateColorMapFromEigenMatrix(const Eigen::MatrixXd &matrix,
                                      bool invert_y = false) {
  cv::Mat grey_img, color_img;

  // scale matrix to 0-255
  Eigen::MatrixXd scaled_matrix =
      (matrix - Eigen::MatrixXd::Ones(matrix.rows(), matrix.cols()) *
                    matrix.minCoeff()) /
      (matrix.maxCoeff() - matrix.minCoeff()) * 255.0;

  Eigen::MatrixXd inverted_matrix =
      Eigen::MatrixXd::Zero(matrix.rows(), matrix.cols());
  if (invert_y) {
    std::size_t row_num = matrix.rows();
    for (std::size_t i = 0; i < row_num; ++i)
      inverted_matrix.row(i) = scaled_matrix.row(row_num - i - 1);
    cv::eigen2cv(inverted_matrix, grey_img);
  } else {
    cv::eigen2cv(scaled_matrix, grey_img);
  }

  grey_img.convertTo(grey_img, CV_8U);
  cv::applyColorMap(grey_img, color_img, cv::COLORMAP_JET);

  return color_img;
}

void ShowMatrixAsImage(const Eigen::MatrixXd &matrix, std::string window_name,
                       bool save_img) {
  cv::Mat img;
  cv::eigen2cv(matrix, img);
  img.convertTo(img, CV_8UC3);
  CvIO::ShowImage(img, window_name, save_img);
}

void ShowMatrixAsColorMap(const Eigen::MatrixXd &matrix,
                          std::string window_name, bool save_img) {
  cv::Mat color_img = CreateColorMapFromEigenMatrix(matrix);
  CvIO::ShowImage(color_img, window_name, save_img);
}

void ShowPathOnMatrixAsColorMap(const Eigen::MatrixXd &matrix,
                                std::vector<RectGridIndex> waypoints,
                                std::string window_name, bool save_img) {
  cv::Mat color_img = CreateColorMapFromEigenMatrix(matrix);

  CvCanvas canvas(color_img);

  for (int i = 0; i < waypoints.size() - 1; ++i) {
    canvas.DrawLine({static_cast<double>(waypoints[i].GetX()),
                     static_cast<double>(waypoints[i].GetY())},
                    {static_cast<double>(waypoints[i + 1].GetX()),
                     static_cast<double>(waypoints[i + 1].GetY())},
                    cv::Scalar(244, 92, 66));
  }

  CvIO::ShowImage(color_img, window_name, save_img);
}
}  // namespace xmotion

#endif /* DENSE_GRID_VISUAL_HPP */
