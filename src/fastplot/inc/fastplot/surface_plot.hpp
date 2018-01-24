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

#include <cmath>

#include <Eigen/Dense>

#include <vtkSmartPointer.h>
#include <vtkStructuredGrid.h>
#include <vtkPointData.h>
#include <vtkFloatArray.h>
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

  // Customization functions
  void SetCameraPosition(double x, double y, double z);
  void SetFocalPosition(double x, double y, double z);
  void EnableAutoScaleRange(bool enable);
  void SetGridEdgeVisibility(bool enable);
  void SetScaleRange(double min, double max);
  void SetWrapScaleFactor(double scale);

  // This function could be called in a loop to show the changing process of a surface
  template <typename DerivedVector1, typename DerivedVector2, typename DerivatedMatrix>
  void ShowSurfaceFrame(const Eigen::MatrixBase<DerivedVector1> &x, const Eigen::MatrixBase<DerivedVector2> &y, const Eigen::MatrixBase<DerivatedMatrix> &z, bool do_warp = false, bool show_box = true, bool show_axes = true, bool show_bar = true)
  {
    // create a grid
    vtkSmartPointer<vtkStructuredGrid> structured_grid = CreateStructuredGrid(x, y, z);

    // render and show in window
    RenderSurface(structured_grid, do_warp, wrap_scale_, show_box, show_axes, show_bar);
    ShowRenderToWindow();
  }

  // This function shows the surface with mouse interactions enabled 
  template <typename DerivedVector1, typename DerivedVector2, typename DerivatedMatrix>
  void ShowSurface(const Eigen::MatrixBase<DerivedVector1> &x, const Eigen::MatrixBase<DerivedVector2> &y, const Eigen::MatrixBase<DerivatedMatrix> &z, bool do_warp = false, bool show_box = true, bool show_axes = true, bool show_bar = true)
  {
    // create a grid
    vtkSmartPointer<vtkStructuredGrid> structured_grid = CreateStructuredGrid(x, y, z);

    // render and show in window
    RenderSurface(structured_grid, do_warp, wrap_scale_, show_box, show_axes, show_bar);
    ShowRenderToWindowWithInteraction();
  }

  // This function renders the surface to a file
  template <typename DerivedVector1, typename DerivedVector2, typename DerivatedMatrix>
  void SaveSurfaceToFile(const Eigen::MatrixBase<DerivedVector1> &x, const Eigen::MatrixBase<DerivedVector2> &y, const Eigen::MatrixBase<DerivatedMatrix> &z, std::string file_name, int32_t pixel_x = 640, int32_t pixel_y = 480, bool do_warp = false, bool show_box = true, bool show_axes = true, bool show_bar = true)
  {
    // create a grid
    vtkSmartPointer<vtkStructuredGrid> structured_grid = CreateStructuredGrid(x, y, z);

    // render and show in window
    RenderSurface(structured_grid, do_warp, wrap_scale_, show_box, show_axes, show_bar);
    SaveRenderToFile(file_name, pixel_x, pixel_y);
  }

protected:
  vtkSmartPointer<vtkRenderer> renderer_;
  vtkSmartPointer<vtkRenderWindow> render_window_;
  vtkSmartPointer<vtkRenderWindowInteractor> render_window_interactor_;

  double wrap_scale_ = 1.0;
  double wrap_scale_factor_ = 1.0;
  double camera_position_[3] = {-1.0, -1.0, 1.0};
  double focal_position_[3] = {0, 0, 0};
  bool z_auto_scale_ = true;
  double z_scale_min_ = 0;
  double z_scale_max_ = 1;
  bool show_grid_edge_ = true;

  void RenderSurface(vtkSmartPointer<vtkStructuredGrid> structured_grid, bool do_warp, double wrap_scale, bool show_box, bool show_axes, bool show_bar);
  void ShowRenderToWindow();
  void ShowRenderToWindowWithInteraction();
  void SaveRenderToFile(std::string file_name, int32_t pixel_x = 640, int32_t pixel_y = 480);

  template <typename DerivedVector1, typename DerivedVector2, typename DerivatedMatrix>
  vtkSmartPointer<vtkStructuredGrid> CreateStructuredGrid(const Eigen::MatrixBase<DerivedVector1> &x, const Eigen::MatrixBase<DerivedVector2> &y, const Eigen::MatrixBase<DerivatedMatrix> &z)
  {
    // Get size of the surface
    const int size_x = x.rows();
    const int size_y = y.rows();
    double wrap_scale = 1.0;

    assert(size_x == z.rows());
    assert(size_y == z.cols());

    double x_range = std::abs(x(0) - x(size_x - 1));
    double y_range = std::abs(y(0) - y(size_y - 1));
    double z_range = std::abs(z.maxCoeff() - z.minCoeff());

    double xy_range = std::min(x_range, y_range);
    wrap_scale_ = xy_range / z_range;

    // setup camera according to data
    focal_position_[0] = x.minCoeff() + x_range / 2.0;
    focal_position_[1] = y.minCoeff() + y_range / 2.0;
    focal_position_[2] = z.minCoeff() + z_range / 2.0;

    // create a grid
    vtkSmartPointer<vtkStructuredGrid> structured_grid =
        vtkSmartPointer<vtkStructuredGrid>::New();

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

    for (unsigned int j = 0; j < size_y; j++)
      for (unsigned int i = 0; i < size_x; i++)
        points->InsertNextPoint(x(i), y(j), z(i, j));

    // Specify the dimensions of the grid
    structured_grid->SetDimensions(size_x, size_y, 1);
    structured_grid->SetPoints(points);

    vtkSmartPointer<vtkFloatArray> colors = vtkSmartPointer<vtkFloatArray>::New();
    colors->SetNumberOfComponents(1);
    colors->SetNumberOfTuples(size_x * size_y);
    int k = 0;
    for (int j = 0; j < size_y; j++)
      for (int i = 0; i < size_x; i++)
      {
        colors->InsertComponent(k, 0, z(i, j));
        k++;
      }
    structured_grid->GetPointData()->SetScalars(colors);

    return structured_grid;
  }
};
}

#endif /* SURFACE_PLOT_HPP */
