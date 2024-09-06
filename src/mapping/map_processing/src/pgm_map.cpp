/*
 * pgm_map.cpp
 *
 * Created on: Jan 28, 2021 21:53
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "map_processing/pgm_map.hpp"

#include "logging/xlogger.hpp"
#include "math_utils/eigen_io.hpp"

#include "pnm/pnm.hpp"

namespace xmotion {
PgmMap::PgmMap(const Metadata& metadata) : metadata_(metadata) {}

bool PgmMap::LoadData() {
  if (metadata_.image.empty()) {
    XLOG_ERROR("Image file path is empty");
    return false;
  }

  XLOG_INFO("Loading PGM image: {}", metadata_.image);
  if (metadata_.image.substr(metadata_.image.size() - 3, 3) != "pgm") {
    XLOG_ERROR("Invalid image file format, only PGM file is supported");
    return false;
  }
  pnm::pgm_image pgm = pnm::read_pgm_binary(metadata_.image);
  if (pgm.size() == 0) {
    XLOG_ERROR("Failed to load PGM image");
    return false;
  }

  auto width = pgm.x_size();
  auto height = pgm.y_size();
  XLOG_INFO("PGM image size: {}x{}", width, height);
  data_ = Eigen::MatrixXi::Zero(height, width);
  for (long y = 0; y < height; ++y) {
    for (long x = 0; x < width; ++x) {
      data_(y, x) = pgm[y][x].value;
    }
  }

  return true;
}

bool PgmMap::LoadFromFile(const std::string& file_path) {
  metadata_.image = file_path;
  return LoadData();
}

bool PgmMap::SaveToFile(const std::string& directory,
                        const std::string& filename) {
  return EigenIO::WriteToImage(directory, filename, data_, true);
}

cv::Mat PgmMap::ToCvMat(int channel_num) {
  // convert data_ to cv::Mat
  if (channel_num == 4) {
    cv::Mat cv_image(data_.rows(), data_.cols(), CV_8UC4);
    for (int y = 0; y < data_.rows(); ++y) {
      for (int x = 0; x < data_.cols(); ++x) {
        cv_image.at<cv::Vec4b>(y, x) =
            cv::Vec4b(data_(y, x), data_(y, x), data_(y, x), 255);
      }
    }
    return cv_image;
  } else if (channel_num == 3) {
    cv::Mat cv_image(data_.rows(), data_.cols(), CV_8UC3);
    for (int y = 0; y < data_.rows(); ++y) {
      for (int x = 0; x < data_.cols(); ++x) {
        cv_image.at<cv::Vec3b>(y, x) =
            cv::Vec3b(data_(y, x), data_(y, x), data_(y, x));
      }
    }
    return cv_image;
  } else if (channel_num == 1) {
    cv::Mat cv_image(data_.rows(), data_.cols(), CV_8UC1);
    for (int y = 0; y < data_.rows(); ++y) {
      for (int x = 0; x < data_.cols(); ++x) {
        cv_image.at<uchar>(y, x) = data_(y, x);
      }
    }
    return cv_image;
  } else {
    XLOG_ERROR("Invalid channel number: {}", channel_num);
    return cv::Mat();
  }
}
}  // namespace xmotion