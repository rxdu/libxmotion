/* 
 * collision_threat.cpp
 * 
 * Created on: Nov 02, 2018 01:40
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "threat_field/collision_threat.hpp"

#include "file_io/folder_path.hpp"

using namespace librav;

CollisionThreat::CollisionThreat(VehicleEstimation est, std::shared_ptr<TrafficChannel> chn) : vehicle_est_(est), traffic_chn_(chn)
{
    SetupPredictionModel();
}

void CollisionThreat::SetupPredictionModel()
{
    // convert vehicle pose info from world frame to path frame
    auto pose_wf = vehicle_est_.GetPose();
    auto pose_pf = traffic_chn_->ConvertToPathCoordinate(SimplePoint(pose_wf.position.x, pose_wf.position.y));

    // instantiate Markov model
    occupancy_ = std::make_shared<MarkovModel>(0, s_max_, 0, v_max_);
    auto s_var_matrix = vehicle_est_.GetPositionVariance();
    double spd_var = vehicle_est_.GetSpeedVariance();

    s_offset_ = pose_pf.s - s_starting_;
    occupancy_->SetupMarkovModel(s_starting_, s_var_matrix(0, 0), vehicle_est_.GetSpeed(), spd_var, true, FolderPath::GetDataFolderPath() + "/reachability/vehicle_threat_combined_state_transition.data");

    // std::cout << "finished setting up markov model" << std::endl;
}

void CollisionThreat::UpdateOccupancyDistribution(int32_t t_k)
{
    occupancy_grid_ = std::make_shared<CurvilinearGrid>(traffic_chn_->center_curve_, s_step_, delta_step_, delta_size_, s_offset_);
    nz_cells_.clear();
    sub_threats_.clear();

    Eigen::VectorXd dist = occupancy_->GetOccupancyDistribution(t_k);

    // std::cout << "dist size: " << dist.size() << " ; " << occupancy_grid_->GetTangentialGridNum() << std::endl;

    double probability_max = 0;
    for (std::size_t i = 0; i < dist.size(); ++i)
    {
        if (i < occupancy_grid_->GetTangentialGridNum())
        {
            for (int32_t j = -occupancy_grid_->GetOneSideGridNumber(); j <= occupancy_grid_->GetOneSideGridNumber(); ++j)
            {
                occupancy_grid_->GetCell(i, j)->extra_attribute = dist(i) * LateralDistribution::GetProbability(j);
                if (occupancy_grid_->GetCell(i, j)->extra_attribute > probability_max)
                    probability_max = occupancy_grid_->GetCell(i, j)->extra_attribute;

                // std::cout << "probability (i,j) " << i << "," << j << " = "
                //           << dist(i) << " * " << LateralDistribution::GetProbability(j) << " = " << occupancy_grid_->GetCell(i, j)->cost_map << std::endl;
            }
        }
    }

    for (std::size_t i = 0; i < dist.size(); ++i)
    {
        if (i < occupancy_grid_->GetTangentialGridNum())
        {
            for (int32_t j = -occupancy_grid_->GetOneSideGridNumber(); j <= occupancy_grid_->GetOneSideGridNumber(); ++j)
            {
                auto cell = occupancy_grid_->GetCell(i, j);

                if (cell->extra_attribute > 0)
                {
                    // Note: extra_attribute is the true probability value
                    nz_cells_.push_back(cell);

                    auto pt = occupancy_grid_->ConvertToCurvePoint(cell->center);
                    sub_threats_.emplace_back(Pose2d(pt.x, pt.y, pt.theta), cell->extra_attribute);

                    // values here are normalized for better visualization
                    cell->cost_map = cell->extra_attribute / probability_max;

                    // std::cout << "probability (i,j) " << i << "," << j << " = " << occupancy_grid_->GetCell(i, j)->cost_map << std::endl;
                }
            }
        }
    }
}