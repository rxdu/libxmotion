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

namespace xmotion {
template <typename T, typename CurveType>
CurvilinearGridBase<T, CurveType>::CurvilinearGridBase(CurveType pcurve,
                                                       double s_step,
                                                       double d_step,
                                                       int32_t d_num,
                                                       double s_offset)
    : curve_(pcurve),
      s_step_(s_step),
      delta_step_(d_step),
      delta_num_(d_num),
      s_offset_(s_offset) {
  SetupGrid(pcurve, s_step, d_step, d_num, s_offset);
}

template <typename T, typename CurveType>
void CurvilinearGridBase<T, CurveType>::SetupGrid(CurveType pcurve,
                                                  double s_step, double d_step,
                                                  int32_t d_num,
                                                  double s_offset) {
  curve_ = pcurve;
  s_step_ = s_step;
  delta_step_ = d_step;
  delta_num_ = d_num;
  s_offset_ = s_offset;

  assert(d_num >= 1);
  delta_half_num_ = delta_num_ / 2;

  if (delta_num_ % 2 == 0)
    center_cell_null_ = true;
  else
    center_cell_null_ = false;

  // defined in s-delta coordinate frame
  // generate knots along centerline
  std::vector<double> sknots;
  for (double s = s_offset_; s <= curve_.GetLength(); s += s_step)
    sknots.push_back(s);

  // std::cout << "s knots added: " << sknots.size() << std::endl;

  assert(sknots.size() > 1);

  int32_t x_idx = 0;
  for (int32_t k = 0; k < sknots.size() - 1; ++k) {
    std::vector<CellType *> rows;
    // generate knots along lateral direction
    for (int32_t i = -delta_half_num_; i <= delta_half_num_; ++i) {
      if (center_cell_null_ && (i == 0)) continue;

      CellType *cell = new CellType(x_idx, i, IndexToID(x_idx, i));
      cell->UpdateGeometry(s_step, d_step, center_cell_null_);
      for (int i = 0; i < 4; ++i)
        cell->vertices[i].position =
            ConvertToGlobalCoordinate(cell->vertices[i]);
      cell->center.position = ConvertToGlobalCoordinate(cell->center);
      rows.push_back(cell);
    }
    ++x_idx;
    grid_tiles_.push_back(rows);
  }
}

template <typename T, typename CurveType>
CurvilinearGridBase<T, CurveType>::~CurvilinearGridBase() {
  for (auto &row : grid_tiles_)
    for (auto &cell : row) delete cell;
}

template <typename T, typename CurveType>
CurvilinearGridBase<T, CurveType>::CurvilinearGridBase(
    const CurvilinearGridBase<T, CurveType> &other) {
  s_step_ = other.s_step_;
  delta_step_ = other.delta_step_;
  delta_num_ = other.delta_num_;
  delta_half_num_ = other.delta_half_num_;
  center_cell_null_ = other.center_cell_null_;
  curve_ = other.curve_;

  for (auto row : other.grid_tiles_) {
    std::vector<CellType *> new_row;
    for (auto cell : row) new_row.push_back(new CellType(*cell));
    grid_tiles_.push_back(new_row);
  }
}

template <typename T, typename CurveType>
CurvilinearGridBase<T, CurveType> &CurvilinearGridBase<T, CurveType>::operator=(
    const CurvilinearGridBase<T, CurveType> &other) {
  CurvilinearGridBase<T, CurveType> tmp(other);
  std::swap(*this, tmp);
  return *this;
}

template <typename T, typename CurveType>
CurvilinearGridBase<T, CurveType>::CurvilinearGridBase(
    CurvilinearGridBase<T, CurveType> &&other) {
  s_step_ = std::move(other.s_step_);
  delta_step_ = std::move(other.delta_step_);
  delta_num_ = std::move(other.delta_num_);
  delta_half_num_ = std::move(other.delta_half_num_);
  center_cell_null_ = std::move(other.center_cell_null_);
  curve_ = std::move(other.curve_);
  grid_tiles_ = std::move(other.grid_tiles_);
}

template <typename T, typename CurveType>
CurvilinearGridBase<T, CurveType> &CurvilinearGridBase<T, CurveType>::operator=(
    CurvilinearGridBase<T, CurveType> &&other) {
  s_step_ = std::move(other.s_step_);
  delta_step_ = std::move(other.delta_step_);
  delta_num_ = std::move(other.delta_num_);
  delta_half_num_ = std::move(other.delta_half_num_);
  center_cell_null_ = std::move(other.center_cell_null_);
  curve_ = std::move(other.curve_);
  grid_tiles_ = std::move(other.grid_tiles_);

  return *this;
}

template <typename T, typename CurveType>
CurvilinearCellBase<T> *CurvilinearGridBase<T, CurveType>::GetCell(int64_t id) {
  auto idx = IDToIndex(id);
  return GetCell(idx);
}

template <typename T, typename CurveType>
CurvilinearCellBase<T> *CurvilinearGridBase<T, CurveType>::GetCell(int32_t x,
                                                                   int32_t y) {
  return grid_tiles_[x][y + delta_half_num_];
}

template <typename T, typename CurveType>
std::vector<CurvilinearCellBase<T> *>
CurvilinearGridBase<T, CurveType>::GetNeighbours(int32_t x, int32_t y,
                                                 int32_t min_h, int32_t max_h) {
  std::vector<CurvilinearCellBase<T> *> neighbours;

  for (int32_t h = min_h; h <= max_h; ++h) {
    if (x + h >= GetTangentialGridNum()) break;

    for (int32_t yi = -delta_half_num_; yi <= delta_half_num_; ++yi) {
      if (center_cell_null_ && yi == 0) continue;
      neighbours.push_back(GetCell(x + h, yi));
    }
  }

  return neighbours;
}

template <typename T, typename CurveType>
std::vector<CurvilinearCellBase<T> *>
CurvilinearGridBase<T, CurveType>::GetNeighbours(int64_t id, int32_t min_h,
                                                 int32_t max_h) {
  auto index = IDToIndex(id);
  return GetNeighbours(index.GetX(), index.GetY(), min_h, max_h);
}

template <typename T, typename CurveType>
SimplePoint2 CurvilinearGridBase<T, CurveType>::ConvertToGlobalCoordinate(
    typename CurvilinearGridBase<T, CurveType>::GridPoint pt) {
  // TODO: calculation could be simplified
  Eigen::Matrix2d rotation_matrix;
  rotation_matrix << 0, -1, 1, 0;

  double s_val = pt.s + s_offset_;
  // auto base_pt = curve_.Evaluate(s_val);
  // auto vel_vec = curve_.Evaluate(s_val, 1);
  double pos_x, pos_y;
  curve_.GetPositionVector(s_val, pos_x, pos_y);
  double vdir_x, vdir_y;
  curve_.GetTangentVector(s_val, vdir_x, vdir_y);

  Eigen::Vector2d base_vec(pos_x, pos_y);
  Eigen::Vector2d vec_t(vdir_x, vdir_y);
  Eigen::Vector2d vec_n = rotation_matrix * vec_t;

  Eigen::Vector2d offset = vec_n.normalized() * pt.delta;
  Eigen::Vector2d result = base_vec + offset;

  return SimplePoint2(result.x(), result.y());
}
}  // namespace xmotion

#endif /* CURVILINEAR_GRID_IMPL_HPP */
