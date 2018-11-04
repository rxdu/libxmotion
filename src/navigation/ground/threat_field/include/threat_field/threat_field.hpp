/* 
 * threat_field.hpp
 * 
 * Created on: Nov 02, 2018 06:02
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef THREAT_FIELD_HPP
#define THREAT_FIELD_HPP

#include <memory>
#include <unordered_map>

#include "threat_field/collision_threat.hpp"

namespace librav
{
class ThreatField
{
  public:
    ThreatField() = default;

    void AddVehicleEstimations(std::vector<VehicleEstimation> ests);

    void SetupThreatField();
    void UpdateThreatField(int32_t t_k);

    std::vector<std::shared_ptr<CollisionThreat>> GetAllCollisionThreats();

  private:
    std::unordered_map<int32_t, VehicleEstimation> vehicles_;
    std::unordered_map<int32_t, std::vector<std::shared_ptr<CollisionThreat>>> threats_;
};
} // namespace librav

#endif /* THREAT_FIELD_HPP */
