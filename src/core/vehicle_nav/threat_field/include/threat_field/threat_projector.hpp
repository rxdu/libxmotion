/* 
 * threat_projector.hpp
 * 
 * Created on: Aug 02, 2018 21:57
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef THREAT_PROJECTOR_HPP
#define THREAT_PROJECTOR_HPP

#include <memory>
#include <string>

#include "road_network/road_map.hpp"

namespace librav
{
class ThreatProjector
{
public:
  ThreatProjector(std::shared_ptr<RoadMap> map);
  ~ThreatProjector() = default;

  void SetRouteStartGoal(std::string start, std::string goal);
  Eigen::MatrixXd GetRouteDrivableMask(bool normalize = false);

private:
  std::shared_ptr<RoadMap> road_map_;

  std::string start_lanelet_;
  std::string goal_lanelet_;
  std::vector<std::string> route_;
};
} // namespace librav

#endif /* THREAT_PROJECTOR_HPP */
