/* 
 * threat_ranking.hpp
 * 
 * Created on: Aug 08, 2018 00:11
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef THREAT_RANKING_HPP
#define THREAT_RANKING_HPP

#include <memory>

#include "road_map/road_map.hpp"
#include "lattice_planner/lattice_planner.hpp"

#include "threat_ranking/topogeo_graph.hpp"
#include "threat_ranking/motion_model.hpp"

namespace librav
{
class ThreatRanking
{
public:
  ThreatRanking(std::shared_ptr<RoadMap> map);

  LatticePath path_;
  std::shared_ptr<MotionModel> motion_model_;

  void AddStateEstimations(std::vector<MMStateEst> ests);

  void SetEgoVehicleFootprint(Polygon fp) { planner_->SetVehicleFootprint(fp); }
  void SetEgoDesiredPath(std::string start, std::string goal);
  void SetEgoStartState(double x, double y, double theta);
  void SetEgoGoalState(double x, double y, double theta);

  void PerformEgoPathPlanning();

  void Analyze();

private:
  std::shared_ptr<RoadMap> road_map_;

  // motion model
  std::shared_ptr<TopoGeoGraph> tg_graph_;

  // planner
  std::shared_ptr<LatticeManager> lattice_manager_;
  std::shared_ptr<LatticePlanner> planner_;

  double start_x_;
  double start_y_;
  double start_theta_;

  double goal_x_;
  double goal_y_;
  double goal_theta_;

  std::vector<std::string> ego_path_;
  std::vector<std::string> active_lanelets_;
};
} // namespace librav

#endif /* THREAT_RANKING_HPP */
