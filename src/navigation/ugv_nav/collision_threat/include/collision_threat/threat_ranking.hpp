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
#include <unordered_map>

#include "road_map/road_map.hpp"
#include "lattice_planner/lattice_planner.hpp"

#include "road_map/topogeo_graph.hpp"
#include "collision_threat/motion_model.hpp"

namespace librav
{
class ThreatRanking
{
public:
  ThreatRanking(std::shared_ptr<RoadMap> map);

  LatticePath path_;
  std::shared_ptr<MotionModel> motion_model_;

  void SetCaseLabel(std::string label) { case_label_ = label; }

  void AddStateEstimations(std::vector<MMStateEst> ests);

  void SetEgoVehicleFootprint(Polygon fp) { planner_->SetVehicleFootprint(fp); }
  void SetEgoDesiredPath(std::string start, std::string goal);
  void SetEgoStartState(double x, double y, double theta);
  void SetEgoGoalState(double x, double y, double theta);

  void PerformEgoPathPlanning();

  void Analyze();
  void PrintCostInfo();

private:
  // road map
  std::shared_ptr<RoadMap> road_map_;

  // planner
  std::shared_ptr<LatticeManager> lattice_manager_;
  std::shared_ptr<LatticePlanner> planner_;

  // scenario information
  double start_x_;
  double start_y_;
  double start_theta_;

  double goal_x_;
  double goal_y_;
  double goal_theta_;

  std::string case_label_ = "ranking";

  std::vector<std::string> ego_path_;
  std::vector<std::string> active_lanelets_;

  std::unordered_map<int32_t, double> threat_cost_;

  void CalculateThreatExposure(const Polyline &line, std::shared_ptr<CollisionField> cfield);
};
} // namespace librav

#endif /* THREAT_RANKING_HPP */
