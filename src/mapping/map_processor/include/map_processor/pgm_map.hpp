/*
 * pgm_map.hpp
 *
 * Created on: Jan 28, 2021 21:47
 * Description: map represented as PGM file (default ROS map format)
 *
 * Reference:
 *  [1] http://davis.lbl.gov/Manuals/NETPBM/doc/pgm.html
 *  [2] https://netpbm.sourceforge.net/doc/pgm.html
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef PGM_MAP_HPP
#define PGM_MAP_HPP

#include <string>
#include <vector>

#include "interface/type/geometry_types.hpp"

namespace xmotion {
class PgmMap {
 public:
  enum class MapMode { kTrinary = 0, kScale, kRaw };

  struct Metadata {
    std::string image;
    MapMode mode;
    double resolution;
    Position3d origin;
    bool negate;
    double occupied_thresh;
    double free_thresh;
  };

  using MapData = Eigen::MatrixXi;

 public:
  PgmMap() = default;
  explicit PgmMap(const Metadata& metadata);

  bool LoadData();
  bool LoadFromFile(const std::string& file_path);
  void SaveToFile(const std::string& file_path);

  Metadata GetMapMetadata() { return metadata_; };

  MapData& GetMapData() { return data_; };

 private:
  Metadata metadata_;
  MapData data_;
};
}  // namespace xmotion

#endif /* PGM_MAP_HPP */
