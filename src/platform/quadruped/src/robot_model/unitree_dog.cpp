/*
 * unitree_dog.cpp
 *
 * Created on 7/6/24 8:51 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/robot_model/unitree_dog.hpp"

namespace xmotion {
UnitreeDog::UnitreeDog(const UnitreeModelProfile& profile)
    : profile_(profile) {}
}  // namespace xmotion