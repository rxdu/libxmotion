/* 
 * field_plot.hpp
 * 
 * Created on: Jan 16, 2018 12:55
 * Description: specialized version of surface plot
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#ifndef FIELD_PLOT_HPP
#define FIELD_PLOT_HPP

#include <cmath>
#include <cstdint>

#include <Eigen/Dense>

#include <vtkSmartPointer.h>
#include <vtkStructuredGrid.h>
#include <vtkPointData.h>
#include <vtkFloatArray.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

#include "fastplot/surface_plot.hpp"

namespace librav
{
class FieldPlot : public SurfacePlot
{
public:
  FieldPlot(int64_t size_x, int64_t size_y);
  FieldPlot() = delete;
  ~FieldPlot() = default;

  // This function could be called in a loop to show the changing process of a surface
  template <typename DerivedVector1, typename DerivedVector2, typename DerivatedMatrix>
  void ShowFieldFrame(const Eigen::MatrixBase<DerivedVector1> &x, const Eigen::MatrixBase<DerivedVector2> &y, const Eigen::MatrixBase<DerivatedMatrix> &z, bool do_warp = false, bool show_box = true, bool show_axes = true, bool show_bar = true)
  {
    // create a grid
    vtkSmartPointer<vtkStructuredGrid> structured_grid = CreateStructuredGrid(x, y, z);

    // render and show in window
    RenderField(structured_grid, do_warp, wrap_scale_, show_box, show_axes, show_bar);
    ShowRenderToWindow();
  }

  // This function shows the surface with mouse interactions enabled 
  template <typename DerivedVector1, typename DerivedVector2, typename DerivatedMatrix>
  void ShowField(const Eigen::MatrixBase<DerivedVector1> &x, const Eigen::MatrixBase<DerivedVector2> &y, const Eigen::MatrixBase<DerivatedMatrix> &z, bool do_warp = false, bool show_box = true, bool show_axes = true, bool show_bar = true)
  {
    // create a grid
    vtkSmartPointer<vtkStructuredGrid> structured_grid = CreateStructuredGrid(x, y, z);

    // render and show in window
    RenderField(structured_grid, do_warp, wrap_scale_, show_box, show_axes, show_bar);
    ShowRenderToWindowWithInteraction();
  }

  // This function renders the surface to a file
  template <typename DerivedVector1, typename DerivedVector2, typename DerivatedMatrix>
  void SaveFieldToFile(const Eigen::MatrixBase<DerivedVector1> &x, const Eigen::MatrixBase<DerivedVector2> &y, const Eigen::MatrixBase<DerivatedMatrix> &z, std::string file_name, int32_t pixel_x = 640, int32_t pixel_y = 480, bool do_warp = false, bool show_box = true, bool show_axes = true, bool show_bar = true)
  {
    // create a grid
    vtkSmartPointer<vtkStructuredGrid> structured_grid = CreateStructuredGrid(x, y, z);

    // render and show in window
    RenderField(structured_grid, do_warp, wrap_scale_, show_box, show_axes, show_bar);
    SaveRenderToFile(file_name, pixel_x, pixel_y);
  }

private:
  int64_t field_size_x_;
  int64_t field_size_y_;

  void RenderField(vtkSmartPointer<vtkStructuredGrid> structured_grid, bool do_warp, double wrap_scale, bool show_box, bool show_axes, bool show_bar);
};
}

#endif /* FIELD_PLOT_HPP */
