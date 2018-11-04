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
    occupancy_ = std::unique_ptr<MarkovModel>(new MarkovModel(0, s_max_, 0, v_max_));

    s_offset_ = pose_pf.s - s_starting_;
    occupancy_->SetupMarkovModel(s_starting_, 2 * 2, vehicle_est_.GetSpeed(), 1 * 1, true, FolderPath::GetDataFolderPath() + "/reachability/vehicle_threat_combined_state_transition.data");
    
    // std::cout << "finished setting up markov model" << std::endl;
}

void CollisionThreat::UpdateOccupancyDistribution(int32_t t_k)
{
    occupancy_grid_ = std::make_shared<CurvilinearGrid>(traffic_chn_->center_curve_, s_step_, delta_step_, delta_size_, s_offset_);

    Eigen::VectorXd dist = occupancy_->GetOccupancyDistribution(t_k);

    // std::cout << "dist size: " << dist.size() << " ; " << occupancy_grid_->GetTangentialGridNum() << std::endl;

    double probability_max = 0;
    for (std::size_t i = 0; i < dist.size(); ++i)
    {
        if (i < occupancy_grid_->GetTangentialGridNum())
        {
            for (int32_t j = -occupancy_grid_->GetOneSideGridNumber(); j <= occupancy_grid_->GetOneSideGridNumber(); ++j)
            {
                occupancy_grid_->GetCell(i, j)->cost_map = dist(i) * LateralDistribution::GetProbability(j);
                if (occupancy_grid_->GetCell(i, j)->cost_map > probability_max)
                    probability_max = occupancy_grid_->GetCell(i, j)->cost_map;
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
                occupancy_grid_->GetCell(i, j)->cost_map = occupancy_grid_->GetCell(i, j)->cost_map / probability_max;
                // std::cout << "probability (i,j) " << i << "," << j << " = " << occupancy_grid_->GetCell(i, j)->cost_map << std::endl;
            }
        }
    }
}