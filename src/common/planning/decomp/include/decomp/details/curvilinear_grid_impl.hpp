/* 
 * curvilinear_grid_impl.hpp
 * 
 * Created on: Oct 20, 2018 10:56
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef CURVILINEAR_GRID_IMPL_HPP
#define CURVILINEAR_GRID_IMPL_HPP

namespace librav
{
template <typename T>
CurvilinearGridBase<T>::CurvilinearGridBase(ParametricCurve pcurve, double s_step, double d_step, int32_t d_num) : curve_(pcurve), s_step_(s_step), delta_step_(d_step), delta_num_(d_num)
{
    assert(d_num >= 1);
    delta_half_num_ = delta_num_ / 2;

    if (delta_num_ % 2 == 0)
        center_cell_null_ = true;
    else
        center_cell_null_ = false;

    // defined in s-delta coordinate frame
    // generate knots along centerline
    std::vector<double> sknots;
    for (double s = 0; s <= curve_.GetTotalLength(); s += s_step)
        sknots.push_back(s);

    int32_t x_idx = 0;
    for (int32_t k = 0; k < sknots.size() - 1; ++k)
    {
        std::vector<CellType *> rows;
        // generate knots along lateral direction
        for (int32_t i = -delta_half_num_; i <= delta_half_num_; ++i)
        {
            if (center_cell_null_ && (i == 0))
                continue;

            CellType *cell = new CellType(x_idx, i, IndexToID(x_idx, i));
            cell->UpdateGeometry(s_step, d_step, center_cell_null_);
            for (int i = 0; i < 4; ++i)
                cell->vertices[i].position = ConvertToGlobalCoordinate(cell->vertices[i]);
            cell->center.position = ConvertToGlobalCoordinate(cell->center);
            rows.push_back(cell);
        }
        ++x_idx;
        grid_tiles_.push_back(rows);
    }
}
template <typename T>
CurvilinearCellBase<T> *CurvilinearGridBase<T>::GetCell(int64_t id)
{
    auto idx = IDToIndex(id);
    return GetCell(idx);
}

template <typename T>
CurvilinearCellBase<T> *CurvilinearGridBase<T>::GetCell(int32_t x, int32_t y)
{
    return grid_tiles_[x][y + delta_half_num_];
}

template <typename T>
std::vector<CurvilinearCellBase<T> *> CurvilinearGridBase<T>::GetNeighbours(int32_t x, int32_t y, int32_t min_h, int32_t max_h)
{
    std::vector<CurvilinearCellBase<T> *> neighbours;

    for (int32_t h = min_h; h <= max_h; ++h)
    {
        if (x + h >= GetTangentialGridNum())
            break;

        for (int32_t yi = -delta_half_num_; yi <= delta_half_num_; ++yi)
        {
            if (center_cell_null_ && yi == 0)
                continue;
            neighbours.push_back(GetCell(x + h, yi));
        }
    }

    return neighbours;
}

template <typename T>
std::vector<CurvilinearCellBase<T> *> CurvilinearGridBase<T>::GetNeighbours(int64_t id, int32_t min_h, int32_t max_h)
{
    auto index = IDToIndex(id);
    return GetNeighbours(index.GetX(), index.GetY(), min_h, max_h);
}

template <typename T>
SimplePoint CurvilinearGridBase<T>::ConvertToGlobalCoordinate(typename CurvilinearGridBase<T>::GridPoint pt)
{
    // TODO: calculation could be simplified
    Eigen::Matrix2d rotation_matrix;
    rotation_matrix << 0, -1, 1, 0;

    auto base_pt = curve_.Evaluate(pt.s);
    auto vel_vec = curve_.Evaluate(pt.s, 1);

    Eigen::Vector2d base_vec(base_pt.x, base_pt.y);
    Eigen::Vector2d vec_t(vel_vec.x, vel_vec.y);
    Eigen::Vector2d vec_n = rotation_matrix * vec_t;

    Eigen::Vector2d offset = vec_n.normalized() * pt.delta;
    Eigen::Vector2d result = base_vec + offset;

    return SimplePoint(result.x(), result.y());
}

template <typename T>
typename CurvilinearGridBase<T>::GridCurvePoint CurvilinearGridBase<T>::ConvertToCurvePoint(GridPoint pt)
{
    // TODO: calculation could be simplified
    Eigen::Matrix2d rotation_matrix;
    rotation_matrix << 0, -1, 1, 0;

    auto base_pt = curve_.Evaluate(pt.s);
    auto vel_vec = curve_.Evaluate(pt.s, 1);
    auto acc_vec = curve_.Evaluate(pt.s, 2);

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

    return GridCurvePoint(result.x(), result.y(), theta_s, kappa_result);
}
} // namespace librav

#endif /* CURVILINEAR_GRID_IMPL_HPP */
