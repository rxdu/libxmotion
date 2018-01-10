/* 
 * surface_plot.hpp
 * 
 * Created on: Jan 10, 2018 14:15
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef SURFACE_PLOT_HPP
#define SURFACE_PLOT_HPP

#include <Eigen/Dense>

#include <vtkSmartPointer.h>
#include <vtkStructuredGrid.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

namespace librav
{
class SurfacePlot
{
  public:
    SurfacePlot();
    ~SurfacePlot() = default;

    // template <typename DerivedVector1, typename DerivedVector2, typename DerivatedMatrix>
    // void ShowSurface(const Eigen::DenseBase<DerivedVector1> &x, const Eigen::DenseBase<DerivedVector2> &y, const Eigen::DenseBase<DerivatedMatrix> &z);
    void ShowSurface(const Eigen::VectorXf &x, const Eigen::VectorXf &y, const Eigen::MatrixXf &z);

  private:
    vtkSmartPointer<vtkStructuredGrid> structured_grid_;
    vtkSmartPointer<vtkRenderer> renderer_;
    vtkSmartPointer<vtkRenderWindow> render_window_;
    vtkSmartPointer<vtkRenderWindowInteractor> render_window_interactor_;

    void RenderSurface();
};
}

#endif /* SURFACE_PLOT_HPP */
