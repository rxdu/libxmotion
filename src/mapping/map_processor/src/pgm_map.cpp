/*
 * pgm_map.cpp
 *
 * Created on: Jan 28, 2021 21:53
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "map_processor/pgm_map.hpp"

#include "logging/xlogger.hpp"

#include "pnm/pnm.hpp"

namespace xmotion {
PgmMap::PgmMap(const Metadata& metadata) : metadata_(metadata) {}

bool PgmMap::LoadData() {
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
  data_ = Eigen::MatrixXi::Zero(height, width);
  for (long y = 0; y < height; ++y) {
    for (long x = 0; x < width; ++x) {
      data_(y, x) = pgm[y][x].value;
    }
  }

  return true;
}

bool PgmMap::LoadFromFile(const std::string& file_path) { return false; }

void SaveToFile(const std::string& file_path) {}
}  // namespace xmotion