/* 
 * curvilinear_grid.hpp
 * 
 * Created on: Oct 20, 2018 09:13
 * Description: this class template provides an implmenetaion of
 *              curvilinear grid based on ParametricCurve by default. 
 * 
 *              If you want to create curvilinear grid with a curve
 *              other than ParametricCurve, you need to have the the
 *              following functions exist:
 * 
 *   double GetLength();
 *   void GetPositionVector(double s, double &x, double &y);
 *   void GetTangentVector(double s, double &x, double &y);
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef CURVILINEAR_GRID_HPP
#define CURVILINEAR_GRID_HPP

#include <cstdint>
#include <vector>
#include <cassert>
#include <iostream>

#include <eigen3/Eigen/Dense>

#include "geometry/simple_point.hpp"
#include "geometry/parametric_curve.hpp"

namespace robotnav
{
/*
 * Coordinate System:
 * 
 *               s
 *               ^
 *               |
 *		         |
 *		         |
 *		        |   s-axis defined by a parametric curve
 *		       |
 *		      |
 *		      |
 *	   <----- o ----- delta
 */

////////////////////////////////////////////////////////////////////

// x - longitudinal/tangential , y - lateral/normal
class CurviGridIndex
{
  public:
    CurviGridIndex() : index_x_(0), index_y_(0) {}
    CurviGridIndex(int64_t x = 0, int64_t y = 0) : index_x_(x), index_y_(y) {}
    ~CurviGridIndex() = default;

    CurviGridIndex(const CurviGridIndex &other) = default;
    CurviGridIndex(CurviGridIndex &&other) = default;
    CurviGridIndex &operator=(const CurviGridIndex &other) = default;
    CurviGridIndex &operator=(CurviGridIndex &&other) = default;

    inline int64_t GetX() const { return index_x_; };
    inline int64_t GetY() const { return index_y_; };
    inline void SetX(int64_t x) { index_x_ = x; };
    inline void SetY(int64_t y) { index_y_ = y; };
    inline void SetXY(int64_t x, int64_t y)
    {
        index_x_ = x;
        index_y_ = y;
    };

  private:
    int64_t index_x_;
    int64_t index_y_;

    friend std::ostream &operator<<(std::ostream &os, const CurviGridIndex &idx)
    {
        os << "(x,y) : " << idx.index_x_ << " , " << idx.index_y_;
        return os;
    }
};

////////////////////////////////////////////////////////////////////

template <typename T>
struct CurvilinearCellBase
{
    struct GridPoint
    {
        GridPoint(double _s = 0, double _d = 0) : s(_s), delta(_d){};
        GridPoint(const GridPoint &other) = default;
        GridPoint(GridPoint &&other) = default;
        GridPoint &operator=(const GridPoint &other) = default;
        GridPoint &operator=(GridPoint &&other) = default;

        double s;
        double delta;

        // for convenient visualization
        SimplePoint position;
    };

    CurvilinearCellBase() = delete;
    CurvilinearCellBase(int32_t xval, int32_t yval, int64_t idval = -1) : index(CurviGridIndex(xval, yval)),
                                                                          id(idval) {}
    ~CurvilinearCellBase() = default;

    CurvilinearCellBase(const CurvilinearCellBase &other) = default;
    CurvilinearCellBase(CurvilinearCellBase &&other) = default;
    CurvilinearCellBase &operator=(const CurvilinearCellBase &other) = default;
    CurvilinearCellBase &operator=(CurvilinearCellBase &&other) = default;

    // for easy reference, maybe unnecessary for some applications
    int64_t id = -1;

    // topological attributes
    CurviGridIndex index;

    // geometrical attributes
    // 4 vertices in the order:
    // 0 - top left, 1 - top right
    // 2 - bottom left, 3 - bottom right
    GridPoint vertices[4];
    GridPoint center;

    // other application specific attributes
    // you can assign values to "cost_map" for visualization, cost_map \in [0,1]
    double cost_map = 0.0;

    // define extra attributes if the default ones are not enough
    T extra_attribute;

    inline int64_t GetUniqueID() const
    {
        return id;
    }

    inline void UpdateGeometry(double s_step, double d_step, bool center_cell_null)
    {
        vertices[0].s = s_step * (index.GetX() + 1);
        vertices[1].s = s_step * (index.GetX() + 1);
        vertices[2].s = s_step * index.GetX();
        vertices[3].s = s_step * index.GetX();

        if (center_cell_null)
        {
            if (index.GetY() >= 0)
                vertices[0].delta = index.GetY() * d_step;
            else
                vertices[0].delta = (index.GetY() + 1) * d_step;
            vertices[1].delta = vertices[0].delta - d_step;
            vertices[2].delta = vertices[0].delta;
            vertices[3].delta = vertices[1].delta;
        }
        else
        {
            if (index.GetY() >= 0)
                vertices[0].delta = index.GetY() * d_step + d_step / 2.0;
            else
                vertices[0].delta = (index.GetY() + 1) * d_step - d_step / 2.0;
            vertices[1].delta = vertices[0].delta - d_step;
            vertices[2].delta = vertices[0].delta;
            vertices[3].delta = vertices[1].delta;
        }

        center.s = vertices[0].s - s_step / 2.0;
        center.delta = (vertices[0].delta + vertices[1].delta) / 2.0;
    }

    inline void PrintInfo() const
    {
        std::cout << "cell " << id << " ; index " << index
                  << " ; center : " << center.s << " , " << center.delta
                  << " ; " << center.position.x << " , " << center.position.y << std::endl;
    }
};

////////////////////////////////////////////////////////////////////

template <typename T, typename CurveType = ParametricCurve>
class CurvilinearGridBase
{
  public:
    struct GridCurvePoint
    {
        GridCurvePoint(double _x = 0, double _y = 0, double _theta = 0, double _kappa = 0) : x(_x), y(_y), theta(_theta), kappa(_kappa){};

        double x;
        double y;
        double theta;
        double kappa;
    };

  public:
    CurvilinearGridBase() = default;
    CurvilinearGridBase(CurveType pcurve, double s_step, double d_step, int32_t d_num, double s_offset = 0);
    virtual ~CurvilinearGridBase();

    CurvilinearGridBase(const CurvilinearGridBase<T, CurveType> &other);
    CurvilinearGridBase &operator=(const CurvilinearGridBase<T, CurveType> &other);
    CurvilinearGridBase(CurvilinearGridBase<T, CurveType> &&other);
    CurvilinearGridBase &operator=(CurvilinearGridBase<T, CurveType> &&other);

    using CellType = CurvilinearCellBase<T>;
    using GridPoint = typename CurvilinearCellBase<T>::GridPoint;

    CurveType curve_;
    std::vector<std::vector<CellType *>> grid_tiles_;

    void SetupGrid(CurveType pcurve, double s_step, double d_step, int32_t d_num, double s_offset = 0);

    /*--------------------------------------------------------------*/

    // local path coordinate
    // - s: starts from 0
    // - delta: positive - left, zero - on curve, negative -right
    SimplePoint ConvertToGlobalCoordinate(GridPoint pt);

    double GetCellSizeS() const { return s_step_; }
    double GetCellSizeDelta() const { return delta_step_; }

    int32_t GetTangentialGridNum() const { return grid_tiles_.size(); }
    int32_t GetNormalGridNum() const { return grid_tiles_.front().size(); }
    int32_t GetOneSideGridNumber() const { return delta_half_num_; }

    inline CurviGridIndex GetIndexFromPathCoordinate(double s, double delta)
    {
        return GridPointToIndex({s, delta});
    }

    inline CurviGridIndex GetIndexFromID(int64_t id) { return IDToIndex(id); }
    inline int64_t GetIDFromIndex(int32_t x, int32_t y) { return IndexToID(x, y); }

    CurvilinearCellBase<T> *GetCell(int64_t id);
    CurvilinearCellBase<T> *GetCell(int32_t x, int32_t y);
    inline CurvilinearCellBase<T> *GetCell(CurviGridIndex idx)
    {
        return grid_tiles_[idx.GetX()][idx.GetY() + delta_half_num_];
    }

    std::vector<CurvilinearCellBase<T> *> GetNeighbours(int32_t x, int32_t y, int32_t min_h, int32_t max_h);
    std::vector<CurvilinearCellBase<T> *> GetNeighbours(int64_t id, int32_t min_h, int32_t max_h);

    void PrintInfo()
    {
        std::cout << "------ Curvilinear Grid ------ " << std::endl;
        for (auto &row : grid_tiles_)
            for (auto &cell : row)
                cell->PrintInfo();
    }

  protected:
    double s_step_;
    double delta_step_;
    int32_t delta_num_;
    double s_offset_;

    int32_t delta_half_num_;
    bool center_cell_null_;

    // IndexToID() and IDToIndex() are the only places that define
    //  the mapping between index and id
    int64_t IndexToID(int32_t x, int32_t y)
    {
        // id is offset by delta_half_num_ so that it starts from 0
        if (!center_cell_null_)
            return x * delta_num_ + y + delta_half_num_;
        else
            return x * (delta_num_ + 1) + y + delta_half_num_;
    }

    inline CurviGridIndex IDToIndex(int64_t id)
    {
        int32_t idx_x, idx_y;

        if (!center_cell_null_)
        {
            idx_x = id / delta_num_;
            idx_y = id % delta_num_ - delta_half_num_;
        }
        else
        {
            idx_x = id / (delta_num_ + 1);
            idx_y = id % (delta_num_ + 1) - delta_half_num_;
        }
        return CurviGridIndex(idx_x, idx_y);
    }

    inline CurviGridIndex GridPointToIndex(GridPoint pt)
    {
        int32_t x = (pt.s - s_offset_) / s_step_;

        int32_t y;
        if (center_cell_null_)
        {
            if (pt.delta == 0)
                y = 0;
            else if (pt.delta > 0)
                y = pt.delta / delta_step_ + 1;
            else if (pt.delta < 0)
                y = pt.delta / delta_step_ - 1;
        }
        else
        {
            if ((pt.delta > -(delta_step_ / 2)) && (pt.delta < (delta_step_ / 2)))
                y = 0;
            else if (pt.delta >= (delta_step_ / 2))
                y = (pt.delta - delta_step_ / 2) / delta_step_ + 1;
            else if (pt.delta <= -(delta_step_ / 2))
                y = (pt.delta + delta_step_ / 2) / delta_step_ - 1;
        }

        return CurviGridIndex(x, y);
    }
};

////////////////////////////////////////////////////////////////////

template <typename T>
class PCurveCurvilinearGrid : public CurvilinearGridBase<T, ParametricCurve>
{
  public:
    PCurveCurvilinearGrid(ParametricCurve pcurve, double s_step, double d_step, int32_t d_num, double s_offset = 0) : CurvilinearGridBase<T, ParametricCurve>(pcurve, s_step, d_step, d_num, s_offset) {}

    // local path coordinate
    // - s: starts from 0
    // - delta: positive - left, zero - on curve, negative -right
    typename CurvilinearGridBase<T, ParametricCurve>::GridCurvePoint ConvertToCurvePoint(typename CurvilinearGridBase<T, ParametricCurve>::GridPoint pt)
    {
        // TODO: calculation could be simplified
        Eigen::Matrix2d rotation_matrix;
        rotation_matrix << 0, -1, 1, 0;

        double s_val = pt.s + CurvilinearGridBase<T, ParametricCurve>::s_offset_;
        auto base_pt = CurvilinearGridBase<T, ParametricCurve>::curve_.Evaluate(s_val);
        auto vel_vec = CurvilinearGridBase<T, ParametricCurve>::curve_.Evaluate(s_val, 1);
        auto acc_vec = CurvilinearGridBase<T, ParametricCurve>::curve_.Evaluate(s_val, 2);

        Eigen::Vector2d base_vec(base_pt.x, base_pt.y);
        Eigen::Vector2d vec_t(vel_vec.x, vel_vec.y);
        Eigen::Vector2d vec_n = rotation_matrix * vec_t;

        // position vector
        Eigen::Vector2d offset = vec_n.normalized() * pt.delta;
        Eigen::Vector2d result = base_vec + offset;

        // theta the same
        double theta_s = std::atan2(vel_vec.y, vel_vec.x);

        // kappa calculation (signed)
        double kappa_s = std::hypot(acc_vec.x, acc_vec.y);
        Eigen::Vector3d vec_acc(acc_vec.x, acc_vec.y, 0);
        Eigen::Vector3d vec_t3(vel_vec.x, vel_vec.y, 0);
        Eigen::Vector3d k_sign_vec = vec_acc.cross(vec_t3);

        double kappa_result;
        if (k_sign_vec(2) >= 0)
            kappa_result = 1.0 / (1.0 / kappa_s + pt.delta);
        else
            kappa_result = 1.0 / (1.0 / kappa_s - pt.delta);

        return typename CurvilinearGridBase<T, ParametricCurve>::GridCurvePoint(result.x(), result.y(), theta_s, kappa_result);
    }
};

using CurvilinearCell = CurvilinearCellBase<double>;
using CurvilinearGrid = PCurveCurvilinearGrid<double>;
} // namespace robotnav

#include "details/curvilinear_grid_impl.hpp"

#endif /* CURVILINEAR_GRID_HPP */
