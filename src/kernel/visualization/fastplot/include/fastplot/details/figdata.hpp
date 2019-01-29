/* 
 * figdata.hpp
 * 
 * Created on: Jan 29, 2019 04:40
 * Description: 
 * 
 * Seminar of Applied Mathematics ETH 2016, D-MATH  
 * https://gitlab.math.ethz.ch/NumCSE/NumCSE/tree/master
 * 
 * Copyright (c) 2016 Julien Gacon <jgacon@ethz.ch>
 *                    Baranidharan Mohan    
 * Copyright (c) 2019 Ruixiang Du (rdu)
 *
 */

#ifndef FIGDATA_HPP
#define FIGDATA_HPP

#include <vector>
#include <type_traits>

#include <eigen3/Eigen/Dense>
#include <mgl2/mgl.h>

namespace librav
{
/// make mglData from std::vector                                   
template <typename Scalar>
typename std::enable_if<std::is_arithmetic<Scalar>::value, mglData>::type
make_mgldata(const std::vector<Scalar> &v)
{
    std::vector<double> vd(v.begin(), v.end());
    return mglData(vd.data(), vd.size());
}

/// make mglData from Eigen::Vector or Eigen::RowVector                     
template <typename Derived>
mglData make_mgldata(const Eigen::MatrixBase<Derived> &vec)
{
    if (!(vec.rows() == 1 || vec.cols() == 1))
    {
        std::cerr << "In function Figure::make_mgldata(): vector.cols() == 1 || vector.rows() == 1 failed!";
        assert(vec.rows() == 1 || vec.cols() == 1);
    }
    std::vector<typename Derived::Scalar> v;
    if (vec.rows() == 1)
    {
        v.resize(vec.cols());
        for (int i = 0; i < vec.cols(); i++)
            v[i] = vec[i];
    }
    else
    {
        v.resize(vec.rows());
        for (int i = 0; i < vec.rows(); i++)
            v[i] = vec[i];
    }
    return mglData(v.data(), v.size());
}

/// make mglData from Eigen::Array
template <typename Derived>
mglData make_mgldata(const Eigen::ArrayBase<Derived> &a)
{
    return make_mgldata(a.matrix());
}
} // namespace librav

#endif /* FIGDATA_HPP */
