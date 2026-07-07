/*
 * dense_grid_visual.hpp
 *
 * Pure image converters for dense grids and Eigen matrices. Module rule:
 * Draw/To functions render onto canvases or return images and NEVER
 * present (no imshow/waitKey) — presentation belongs to SimViewer2D or
 * explicit CvIO calls at the application edge, which keeps every converter
 * usable headless (tests, file export, CI artifacts).
 *
 * Reference:
 * [1] https://www.learnopencv.com/applycolormap-for-pseudocoloring-in-opencv-c-python/
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef DENSE_GRID_VISUAL_HPP
#define DENSE_GRID_VISUAL_HPP

#include <vector>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "cvdraw/cvdraw.hpp"

#include "xmnav/decomp/dense_grid.hpp"

namespace xmotion {

inline cv::Mat MatrixToImage(const Eigen::MatrixXd &matrix) {
  cv::Mat img;
  cv::eigen2cv(matrix, img);
  img.convertTo(img, CV_8UC3);
  return img;
}

inline cv::Mat MatrixToColorMap(const Eigen::MatrixXd &matrix,
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

// kept under its historical name for existing call sites
inline cv::Mat CreateColorMapFromEigenMatrix(const Eigen::MatrixXd &matrix,
                                             bool invert_y = false) {
  return MatrixToColorMap(matrix, invert_y);
}

inline cv::Mat PathOnColorMap(const Eigen::MatrixXd &matrix,
                              const std::vector<RectGridIndex> &waypoints) {
  cv::Mat color_img = MatrixToColorMap(matrix);
  quickviz::CvCanvas canvas(color_img);
  for (int i = 0; i < static_cast<int>(waypoints.size()) - 1; ++i) {
    canvas.DrawLine({static_cast<double>(waypoints[i].GetX()),
                     static_cast<double>(waypoints[i].GetY())},
                    {static_cast<double>(waypoints[i + 1].GetX()),
                     static_cast<double>(waypoints[i + 1].GetY())},
                    cv::Scalar(244, 92, 66));
  }
  return color_img;
}

inline cv::Mat DenseGridToImage(const DenseGrid &grid) {
  return MatrixToImage(grid.GetGridMatrix(true) * 128);
}

inline cv::Mat DenseGridToColorMap(const DenseGrid &grid) {
  return MatrixToColorMap(grid.GetGridMatrix(true));
}

inline cv::Mat PathOnDenseGrid(const DenseGrid &grid,
                               const std::vector<RectGridIndex> &waypoints) {
  return PathOnColorMap(grid.GetGridMatrix(true), waypoints);
}

}  // namespace xmotion

#endif /* DENSE_GRID_VISUAL_HPP */
