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
CurvilinearGridBase<T>::CurvilinearGridBase(ParametricCurve pcurve, double s_step, double d_step) : curve_(pcurve), s_step_(s_step), delta_step_(d_step)
{
}
} // namespace librav

#endif /* CURVILINEAR_GRID_IMPL_HPP */
