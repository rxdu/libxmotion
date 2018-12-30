/* 
 * realvector_space.hpp
 * 
 * Created on: Dec 29, 2018 06:29
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef REALVECTOR_SPACE_HPP
#define REALVECTOR_SPACE_HPP

#include <cstdint>
#include <cassert>
#include <vector>
#include <unordered_map>

#include "sampling/details/space/space_base.hpp"
#include "sampling/details/space/realvector_bound.hpp"

namespace librav
{
template <int32_t N>
class RealVectorSpace : public SpaceBase
{
  public:
    class State : public StateBase
    {

    };

    public : RealVectorSpace()
    {
        bounds_.resize(N);
    }

    RealVectorSpace(std::initializer_list<RealVectorBound> bounds)
    {
        assert(bounds.size() == N);
        bounds_.resize(N);
        int32_t i = 0;
        for (auto &bd : bounds)
            bounds_[i++] = bd;
    }

    void SetBound(int32_t dim, double min, double max)
    {
        assert(dim < N);
        bounds_[dim] = RealVectorBound(min, max);
    }

    int32_t GetDimension() const override { return N; }
    std::vector<RealVectorBound> GetBounds() const { return bounds_; }

    RealVectorBound GetBound(int32_t dim) const
    {
        assert(dim < N);
        return bounds_[dim];
    }

    double GetVolume() const
    {
        double vol = 1.0;
        for (auto &dim_bd : bounds_)
            vol *= dim_bd.GetDifference();
        return vol;
    }

    StateBase *SampleUniform() override{};
    StateBase *SampleUniformNear(const StateBase *near, double distance) override{};
    StateBase *SampleGaussian(const StateBase *mean, double stdDev) override{};

    void PrintInfo()
    {
        std::cout << "dimension: " << GetDimension() << std::endl;
        std::cout << "bounds: " << std::endl;
        for (int32_t i = 0; i < N; ++i)
            std::cout << " - dim " << i << " : [ " << bounds_[i].GetLow() << " , " << bounds_[i].GetHigh() << " ]" << std::endl;
    }

  private:
    std::vector<RealVectorBound> bounds_;
};
} // namespace librav

#endif /* REALVECTOR_SPACE_HPP */
