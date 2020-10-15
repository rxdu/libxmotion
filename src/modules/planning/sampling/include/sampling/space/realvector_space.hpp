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
#include <atomic>
#include <vector>
#include <cmath>
#include <iostream>
#include <unordered_map>

#include <eigen3/Eigen/Dense>

#include "sampling/base/space_base.hpp"
#include "sampling/space/realvector_bound.hpp"

namespace ivnav
{
template <int32_t N>
class RealVectorSpace : public SpaceBase
{
    using BaseType = SpaceBase;

  public:
    class StateType : public State
    {
        StateType() : State(StateType::count)
        {
            StateType::count.fetch_add(1);
        };

        StateType(std::initializer_list<double> vals) : State(StateType::count)
        {
            int32_t i = 0;
            for (auto &val : vals)
                values_[i++] = val;
            StateType::count.fetch_add(1);
        }

        template <int32_t M>
        friend class RealVectorSpace;

      public:
        double values_[N];

        double operator[](int32_t i) const override
        {
            assert(i < N);
            return values_[i];
        }

        double &operator[](int32_t i) override
        {
            assert(i < N);
            return values_[i];
        }

        friend std::ostream &operator<<(std::ostream &os, const StateType &other)
        {
            os << "[ ";
            for (int i = 0; i < N; ++i)
                os << other.values_[i] << " ";
            os << "]";
            return os;
        }

        // statistics
        static std::atomic<std::size_t> count;
    };

  public:
    RealVectorSpace()
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

    ~RealVectorSpace()
    {
        for (auto &state : all_states_)
            delete state.second;
    }

    RealVectorSpace(RealVectorSpace &&other)
    {
        bounds_ = std::move(other.bounds_);
        all_states_ = std::move(other.all_states_);
        // to avoid the destructor free memory multiple times
        for (auto &state : other.all_states_)
            state.second = nullptr;
    }

    RealVectorSpace &operator=(RealVectorSpace &&other)
    {
        for (auto &state : all_states_)
            delete state.second;
        this->bounds_ = std::move(other.bounds_);
        this->all_states_ = std::move(other.all_states_);
        for (auto &state : other.all_states_)
            state.second = nullptr;
        return *this;
    }

    void SetBound(int32_t dim, double min, double max)
    {
        assert(dim < N);
        bounds_[dim] = RealVectorBound(min, max);
    }

    static constexpr int32_t DimensionSize = N;
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

    double EvaluateDistance(const State *sstate, const State *dstate) override
    {
        const StateType *first = static_cast<const StateType *>(sstate);
        const StateType *second = static_cast<const StateType *>(dstate);

        // 2-norm distance
        double se_sum = 0;
        for (int i = 0; i < N; ++i)
        {
            double err = first->values_[i] - second->values_[i];
            se_sum += err * err;
        }
        return std::sqrt(se_sum);
    }

    StateType *CreateState(std::initializer_list<double> elements) override
    {
        assert(elements.size() >= N);

        StateType *new_state = new StateType();
        int i = 0;
        for (auto &element : elements)
            new_state->values_[i++] = element;
        all_states_[new_state->id_] = new_state;
        return new_state;
    }

    StateType *CreateState(const Eigen::MatrixXd &elements) override
    {
        assert(elements.size() >= N);

        StateType *new_state = new StateType();
        for (int i = 0; i < N; ++i)
            new_state->values_[i] = elements(i);
        all_states_[new_state->id_] = new_state;
        return new_state;
    }

    StateType *SampleUniform() override
    {
        StateType *new_state = new StateType();
        for (int i = 0; i < N; ++i)
            new_state->values_[i] = BaseType::rng_.UniformReal(bounds_[i].GetLow(), bounds_[i].GetHigh());
        all_states_[new_state->id_] = new_state;
        return new_state;
    };

    StateType *SampleUniformNear(const State *near, double distance) override
    {
        const StateType *near_state = static_cast<const StateType *>(near);
        StateType *new_state = new StateType();
        for (int i = 0; i < N; ++i)
            new_state->values_[i] = BaseType::rng_.UniformReal(std::max(bounds_[i].GetLow(), near_state->values_[i] - distance),
                                                               std::min(bounds_[i].GetHigh(), near_state->values_[i] + distance));
        all_states_[new_state->id_] = new_state;
        return new_state;
    };

    StateType *SampleGaussian(const State *mean, double stdDev) override
    {
        const StateType *mean_state = static_cast<const StateType *>(mean);
        StateType *new_state = new StateType();
        for (int i = 0; i < N; ++i)
        {
            double v = BaseType::rng_.Gaussian(mean_state->values_[i], stdDev);
            if (v < bounds_[i].GetLow())
                v = bounds_[i].GetLow();
            else if (v > bounds_[i].GetHigh())
                v = bounds_[i].GetHigh();
            new_state->values_[i] = v;
        }
        all_states_[new_state->id_] = new_state;
        return new_state;
    };

    void PrintInfo()
    {
        std::cout << "dimension: " << GetDimension() << std::endl;
        std::cout << "bounds: " << std::endl;
        for (int32_t i = 0; i < N; ++i)
            std::cout << " - dim " << i << " : [ " << bounds_[i].GetLow() << " , " << bounds_[i].GetHigh() << " ]" << std::endl;
    }

  private:
    std::vector<RealVectorBound> bounds_;
    std::unordered_map<std::size_t, StateType *> all_states_;
};

template <int32_t N>
std::atomic<std::size_t> RealVectorSpace<N>::StateType::count = {0};
} // namespace ivnav

#endif /* REALVECTOR_SPACE_HPP */
