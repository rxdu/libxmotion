/*
 * pgm_map.hpp
 *
 * Created on: Jan 28, 2021 21:47
 * Description: map represented as PGM file (default ROS map format)
 *
 * Reference:
 *  [1] http://davis.lbl.gov/Manuals/NETPBM/doc/pgm.html
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef PGM_MAP_HPP
#define PGM_MAP_HPP

#include <string>
#include <vector>

#include <eigen3/Eigen/Core>

namespace robosw {
class PgmMap {
 public:
  struct MapInfo {
    std::string file;
    double resolution;
    std::vector<double> origin;
    bool negate;
    double occupied_thresh;
    double free_thresh;
  };

  using MapData = Eigen::MatrixXi;

 public:
  PgmMap() = default;
  explicit PgmMap(std::string yaml_file);

  bool LoadMapData();

  MapInfo GetMapInfo() { return info_; };
  MapData& GetMapData() { return data_; };

 private:
  MapInfo info_;
  MapData data_;
};
}  // namespace robosw

#endif /* PGM_MAP_HPP */
