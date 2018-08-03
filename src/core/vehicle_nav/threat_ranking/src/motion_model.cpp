/* 
 * motion_model.cpp
 * 
 * Created on: Aug 03, 2018 11:42
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#include "threat_ranking/motion_model.hpp"

using namespace librav;

MotionModel::MotionModel(std::shared_ptr<RoadMap> map):road_map_(map)
{
    
}