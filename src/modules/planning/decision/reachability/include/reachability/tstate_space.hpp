/* 
 * tstate_space.hpp
 * 
 * Created on: Oct 29, 2018 22:31
 * Description: tangential state space (s-v)
 * 
 * Reference:
 *  [1] https://www.ezbordercrossing.com/travel-resources/speed-limits-u-s-and-canada/
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TSTATE_SPACE_HPP
#define TSTATE_SPACE_HPP

#include <cstdint>
#include <vector>
#include <iostream>
#include <cassert>
// #include <unordered_map>

namespace ivnav
{
class TStateSpace
{
  public:
    struct State
    {
        State() = default;
        State(double _s, double _v) : s(_s), v(_v) {}

        double s = 0;
        double v = 0;

        friend std::ostream &operator<<(std::ostream &os, const State &state)
        {
            os << "state: (" << state.s << " , " << state.v << ")";
            return os;
        }
    };

    struct StateCell
    {
        StateCell() = default;
        StateCell(int32_t _id, double smin, double smax, double vmin, double vmax) : id(_id), cs_min(smin), cs_max(smax), cv_min(vmin), cv_max(vmax) {}

        int32_t id;
        double cs_min = 0;
        double cs_max = 0;
        double cv_min = 0;
        double cv_max = 0;

        // used for simulation
        int32_t occupancy_stats;

        // used for probability calculation
        double probability;

        double GetSMiddle() const { return (cs_min + (cs_max - cs_min) / 2.0); }
        double GetVMiddle() const { return (cv_min + (cv_max - cv_min) / 2.0); }

        std::vector<State> GetUniformSamples(int32_t ssize, int32_t vsize)
        {
            double sstep = (cs_max - cs_min) / (ssize + 1);
            double vstep = (cv_max - cv_min) / (vsize + 1);

            std::vector<State> samples;
            for (int32_t i = 1; i <= ssize; ++i)
                for (int32_t j = 1; j <= vsize; ++j)
                    samples.emplace_back(cs_min + i * sstep, cv_min + j * vstep);
            return samples;
        }

        void PrintInfo()
        {
            std::cout << "cell " << id << " : (" << cs_min << " , " << cv_min << ") - (" << cs_max << " , " << cv_max << ")" << std::endl;
        }
    };

  public:
    TStateSpace(double smin = 0, double smax = 0, double vmin = 0, double vmax = 0) : s_min_(smin), s_max_(smax), v_min_(vmin), v_max_(vmax) {}
    ~TStateSpace()
    {
        for (auto &row : state_cells_)
            for (auto cell : row)
                delete cell;
    }

    // copy and assignment of a state space is not allowed
    TStateSpace(const TStateSpace &other) = delete;
    TStateSpace &operator=(const TStateSpace &other) = delete;

    int32_t control_set_size_ = 0;
    int32_t sim_number_per_ctrl_ = 0;

    void SetRange(double smin, double smax, double vmin, double vmax)
    {
        s_min_ = smin;
        s_max_ = smax;
        v_min_ = vmin;
        v_max_ = vmax;
    }

    int32_t GetSSize() const { return s_size_; }
    int32_t GetVSize() const { return v_size_; }
    double GetSStep() const { return s_step_; }
    double GetVStep() const { return v_step_; }
    int32_t GetStateNumber() const { return s_size_ * v_size_; }

    StateCell *GetStateCell(double s, double v)
    {
        int32_t s_idx = s / s_step_;
        int32_t v_idx = v / v_step_;

        if ((s_idx < 0) || (s_idx >= s_size_) || (v_idx < 0) || (v_idx >= v_size_))
            return nullptr;

        return state_cells_[s_idx][v_idx];
    }

    std::vector<std::vector<StateCell *>> GetAllStateCells() { return state_cells_; }

    std::vector<StateCell *> GetStateCellsByS(double s)
    {
        int32_t s_idx = s / s_step_;
        assert(s_idx < s_size_);

        return state_cells_[s_idx];
    }

    // sstep, vstep: incremental length
    void DiscretizeSpace(double sstep, double vstep)
    {
        s_step_ = sstep;
        v_step_ = vstep;
        s_size_ = (s_max_ - s_min_) / s_step_;
        v_size_ = (v_max_ - v_min_) / v_step_;

        // std::cout << "state space discretization size: " << s_size_ << " , " << v_size_ << std::endl;

        DiscretizeSpace();

        // std::cout << "cell size: " << state_cells_.size() << " , " << state_cells_.front().size() << std::endl;
    }

    // ssize, vsize: number of intervals
    void DiscretizeSpaceBySize(int32_t ssize, int32_t vsize)
    {
        s_step_ = (s_max_ - s_min_) / ssize;
        v_step_ = (v_max_ - v_min_) / vsize;
        s_size_ = ssize;
        v_size_ = vsize;

        // std::cout << "state space discretization size: " << s_size_ << " , " << v_size_ << std::endl;

        DiscretizeSpace();

        // std::cout << "cell size: " << state_cells_.size() << " , " << state_cells_.front().size() << std::endl;
    }

  private:
    double s_min_;
    double s_max_;
    double v_min_;
    double v_max_;

    // discretization params
    double s_step_;
    double v_step_;
    int32_t s_size_;
    int32_t v_size_;

    // cells saved as pointer because we need to store info back
    std::vector<std::vector<StateCell *>> state_cells_;

    inline void DiscretizeSpace()
    {
        int32_t cell_count = 0;
        state_cells_.clear();
        // state_cells_.reserve(s_size_);
        for (int32_t i = 0; i < s_size_; ++i)
        {
            std::vector<StateCell *> cells;
            // cells.reserve(v_size_);
            double smin = s_min_ + i * s_step_;
            double smax = s_min_ + (i + 1) * s_step_;
            for (int32_t j = 0; j < v_size_; ++j)
                cells.push_back(new StateCell(cell_count++, smin, smax, v_min_ + j * v_step_, v_min_ + (j + 1) * v_step_));
            state_cells_.push_back(cells);
        }
    }
};
} // namespace ivnav

#endif /* TSTATE_SPACE_HPP */
