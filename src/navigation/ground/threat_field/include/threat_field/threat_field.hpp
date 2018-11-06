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

    double operator()(double x, double y)
    {
        double threat = 0.0;
        for (auto &threat_entry : threats_)
            for (auto sub : threat_entry.second)
                threat += (*sub.get())(x, y);
        return threat;
    }

    Point2d GetThreatCenter()
    {
        Point2d pos(0, 0);
        for (auto &threat_entry : threats_)
            for (auto sub : threat_entry.second)
            {
                auto c = sub->GetThreatCenter();
                pos.x += c.x;
                pos.y += c.y;
            }
        pos.x = pos.x / threats_.size();
        pos.y = pos.y / threats_.size();
        return pos;
    }

    double operator()(double x, double y, int32_t t_k)
    {
        double threat = 0.0;
        for (auto &threat_entry : threats_)
            for (auto sub : threat_entry.second)
                threat += (*sub.get())(x, y, t_k);
        return threat;
    }

    Point2d GetThreatCenter(int32_t t_k)
    {
        Point2d pos(0, 0);
        for (auto &threat_entry : threats_)
            for (auto sub : threat_entry.second)
            {
                auto c = sub->GetThreatCenter(t_k);
                pos.x += c.x;
                pos.y += c.y;
            }
        pos.x = pos.x / threats_.size();
        pos.y = pos.y / threats_.size();
        return pos;
    }

  private:
    std::unordered_map<int32_t, VehicleEstimation> vehicles_;
    std::unordered_map<int32_t, std::vector<std::shared_ptr<CollisionThreat>>> threats_;
};
} // namespace librav

#endif /* THREAT_FIELD_HPP */
