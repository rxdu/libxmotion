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
    if (speed_profile_ != nullptr)
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

void ReferenceTrajectory::FindPathSegment(const double s, StateLattice &lattice, double &s_offset)
{
    assert(s < total_length_);

    double accumulated = 0;
    for (auto &seg : path_)
    {
        if (accumulated + seg.GetLength() > s)
        {
            lattice = seg;
            s_offset = s - accumulated;
            return;
        }
        accumulated += seg.GetLength();
    }
}

double ReferenceTrajectory::AccumulatedDistance(double t)
{
    // TODO fix this function for non-const speed profiles
    return speed_profile_->GetSpeed(t) * t;
}

ReferenceTrajectory::RefPoint ReferenceTrajectory::GetDesiredState(double t)
{
    double dist = AccumulatedDistance(t);

    // double accumulated = 0;
    // for (auto &lattice : path_)
    // {
    //     if (accumulated + lattice.GetLength() > dist)
    //     {
    //         double sf = dist - accumulated;
    //         MotionState ms = lattice.Evaluate(sf);

    //         return RefPoint(ms.x, ms.y, ms.theta, ms.kappa, speed_profile_->GetSpeed(t), speed_profile_->GetAccel(t));
    //     }
    //     accumulated += lattice.GetLength();
    // }

    // return RefPoint();

    StateLattice path_seg;
    double s_offset;
    FindPathSegment(dist, path_seg, s_offset);
    MotionState ms = path_seg.Evaluate(s_offset);

    return RefPoint(ms.x, ms.y, ms.theta, ms.kappa, speed_profile_->GetSpeed(t), speed_profile_->GetAccel(t));
}

void ReferenceTrajectory::GetPositionVector(double s, double &x, double &y)
{
    StateLattice path_seg;
    double s_offset;
    FindPathSegment(s, path_seg, s_offset);

    path_seg.GetPositionVector(s_offset, x, y);
}

void ReferenceTrajectory::GetTangentVector(double s, double &x, double &y)
{
    StateLattice path_seg;
    double s_offset;
    FindPathSegment(s, path_seg, s_offset);

    path_seg.GetTangentVector(s_offset, x, y);
}