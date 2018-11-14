/* 
 * reference_trajectory.cpp
 * 
 * Created on: Nov 14, 2018 03:31
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "local_planner/reference_trajectory.hpp"

using namespace librav;

ReferenceTrajectory::ReferenceTrajectory(std::vector<StateLattice> path) : path_(path)
{
}

ReferenceTrajectory::~ReferenceTrajectory()
{
    delete speed_profile_;
}

ReferenceTrajectory::ReferenceTrajectory(const ReferenceTrajectory &other) : path_(other.path_),
                                                                             total_length_(other.total_length_)
{
    this->speed_profile_ = other.speed_profile_->GetCopy();
}

ReferenceTrajectory &ReferenceTrajectory::operator=(const ReferenceTrajectory &other)
{
    ReferenceTrajectory temp(other);
    *this = std::move(temp);
    return *this;
}

ReferenceTrajectory::ReferenceTrajectory(ReferenceTrajectory &&other) : path_(other.path_),
                                                                        total_length_(other.total_length_)
{
    this->speed_profile_ = std::move(other.speed_profile_);
}

ReferenceTrajectory &ReferenceTrajectory::operator=(ReferenceTrajectory &&other)
{
    this->path_ = std::move(other.path_);
    this->total_length_ = other.total_length_;
    this->speed_profile_ = std::move(other.speed_profile_);

    return *this;
}

void ReferenceTrajectory::GenerateConstantSpeedProfile(double speed)
{
    total_length_ = 0;
    for (auto &lattice : path_)
        total_length_ += lattice.GetLength();
    speed_profile_ = new ConstSpeedProfile(speed);
    std::cout << "total length: " << total_length_ << std::endl;
}

double ReferenceTrajectory::AccumulatedDistance(double t)
{
    // TODO fix this function for non-const speed profiles
    return speed_profile_->GetSpeed(t) * t;
}

ReferenceTrajectory::RefPoint ReferenceTrajectory::GetDesiredState(double t)
{
    double dist = AccumulatedDistance(t);
    double accumulated = 0;
    for (auto &lattice : path_)
    {
        if (accumulated + lattice.GetLength() > dist)
        {
            double sf = dist - accumulated;
            MotionState ms = lattice.Evaluate(sf);

            return RefPoint(ms.x, ms.y, ms.theta, ms.kappa, speed_profile_->GetSpeed(t), speed_profile_->GetAccel(t));
        }
        accumulated += lattice.GetLength();
    }

    // return zero point if failed to find one
    return RefPoint();
}
