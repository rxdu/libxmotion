/* 
 * mathgl_field.hpp
 * 
 * Created on: Mar 15, 2018 15:45
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef MATHGL_FIELD_HPP
#define MATHGL_FIELD_HPP

#include "fastplot/mathgl_2d.hpp"

namespace librav
{
namespace FastPlot
{
template <typename DerivedVector1, typename DerivedVector2, typename DerivatedMatrix>
void FieldSurf(const Eigen::MatrixBase<DerivedVector1> &x, const Eigen::MatrixBase<DerivedVector2> &y, const Eigen::MatrixBase<DerivatedMatrix> &z)
{
    MathGLSurf surf;
    surf.SetData('z', z);
    surf.SetRangeDataX(x);
    surf.SetRangeDataY(y);

    MathGLPlot::Run(&surf);
}

template <typename DerivedVector1, typename DerivedVector2, typename DerivatedMatrix>
void FieldMesh(const Eigen::MatrixBase<DerivedVector1> &x, const Eigen::MatrixBase<DerivedVector2> &y, const Eigen::MatrixBase<DerivatedMatrix> &z)
{
    MathGLMesh mesh;
    mesh.SetData('z', z);
    mesh.SetRangeDataX(x);
    mesh.SetRangeDataY(y);

    MathGLPlot::Run(&mesh);
}

template <typename DerivedVector1, typename DerivedVector2, typename DerivatedMatrix>
void FieldDens(const Eigen::MatrixBase<DerivedVector1> &x, const Eigen::MatrixBase<DerivedVector2> &y, const Eigen::MatrixBase<DerivatedMatrix> &z)
{
    MathGLDens dens;
    dens.SetData('z', z);
    dens.SetRangeDataX(x);
    dens.SetRangeDataY(y);

    MathGLPlot::Run(&dens);
}
}
}

#endif /* MATHGL_FIELD_HPP */
