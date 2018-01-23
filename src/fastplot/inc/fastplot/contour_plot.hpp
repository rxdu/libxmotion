/* 
 * contour_plot.hpp
 * 
 * Created on: Jan 23, 2018 17:07
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef CONTOUR_PLOT_HPP
#define CONTOUR_PLOT_HPP

#include <cmath>

#include <Eigen/Dense>

#include <vtkSmartPointer.h>
#include <vtkRectilinearGrid.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkFloatArray.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

namespace librav
{
class ContourPlot
{
public:
  ContourPlot();
  ~ContourPlot() = default;

  // Customization functions
  void SetCameraPosition(double x, double y, double z);
  void SetFocalPosition(double x, double y, double z);

  // This function could be called in a loop to show the changing process of a surface
  template <typename DerivedVector1, typename DerivedVector2, typename DerivatedMatrix>
  void ShowContourFrame(const Eigen::MatrixBase<DerivedVector1> &x, const Eigen::MatrixBase<DerivedVector2> &y, const Eigen::MatrixBase<DerivatedMatrix> &z, bool show_surf = false, bool show_box = true, bool show_axes = true, bool show_bar = true)
  {
    // create a grid
    vtkSmartPointer<vtkRectilinearGrid> rectilinear_grid = CreateRectilinearGrid(x, y, z);

    // render and show in window
    RenderContour(rectilinear_grid, show_surf, line_num_, show_box, show_axes, show_bar);
    ShowRenderToWindow();
  }

  // This function shows the surface with mouse interactions enabled
  template <typename DerivedVector1, typename DerivedVector2, typename DerivatedMatrix>
  void ShowContour(const Eigen::MatrixBase<DerivedVector1> &x, const Eigen::MatrixBase<DerivedVector2> &y, const Eigen::MatrixBase<DerivatedMatrix> &z, bool show_surf = false, bool show_box = true, bool show_axes = true, bool show_bar = true)
  {
    // create a grid
    vtkSmartPointer<vtkRectilinearGrid> rectilinear_grid = CreateRectilinearGrid(x, y, z);

    // render and show in window
    RenderContour(rectilinear_grid, show_surf, line_num_, show_box, show_axes, show_bar);
    ShowRenderToWindowWithInteraction();
  }

  // This function renders the surface to a file
  template <typename DerivedVector1, typename DerivedVector2, typename DerivatedMatrix>
  void SaveContourToFile(const Eigen::MatrixBase<DerivedVector1> &x, const Eigen::MatrixBase<DerivedVector2> &y, const Eigen::MatrixBase<DerivatedMatrix> &z, std::string file_name, int32_t pixel_x = 640, int32_t pixel_y = 480, bool show_surf = false, bool show_box = true, bool show_axes = true, bool show_bar = true)
  {
    // create a grid
    vtkSmartPointer<vtkRectilinearGrid> rectilinear_grid = CreateRectilinearGrid(x, y, z);

    // render and show in window
    RenderContour(rectilinear_grid, show_surf, line_num_, show_box, show_axes, show_bar);
    SaveRenderToFile(file_name, pixel_x, pixel_y);
  }

protected:
  vtkSmartPointer<vtkRenderer> renderer_;
  vtkSmartPointer<vtkRenderWindow> render_window_;
  vtkSmartPointer<vtkRenderWindowInteractor> render_window_interactor_;

  int line_num_ = 20;
  double camera_position_[3] = {0.0, 0.0, 1.0};
  double focal_position_[3] = {0, 0, 0};

  void RenderContour(vtkSmartPointer<vtkRectilinearGrid> rectilinear_grid, bool draw_surf, int line_num, bool show_box, bool show_axes, bool show_bar);
  void ShowRenderToWindow();
  void ShowRenderToWindowWithInteraction();
  void SaveRenderToFile(std::string file_name, int32_t pixel_x = 640, int32_t pixel_y = 480);

  template <typename DerivedVector1, typename DerivedVector2, typename DerivatedMatrix>
  vtkSmartPointer<vtkRectilinearGrid> CreateRectilinearGrid(const Eigen::MatrixBase<DerivedVector1> &x, const Eigen::MatrixBase<DerivedVector2> &y, const Eigen::MatrixBase<DerivatedMatrix> &z)
  {
    // Get size of the surface
    const int size_x = x.rows();
    const int size_y = y.rows();

    // std::cout << "size x-y-z: " << size_x << " , " << size_y << " , " << z.rows() << " * " << z.cols() << std::endl;

    assert(size_x == z.rows());
    assert(size_y == z.cols());

    double x_range = std::abs(x(0) - x(size_x - 1));
    double y_range = std::abs(y(0) - y(size_y - 1));
    double z_range = std::abs(z.maxCoeff() - z.minCoeff());

    // double xy_range = std::min(x_range, y_range);

    // setup camera according to data
    camera_position_[0] = x.minCoeff() + x_range / 2.0;
    camera_position_[1] = y.minCoeff() + y_range / 2.0;
    camera_position_[2] = 1;
    focal_position_[0] = x.minCoeff() + x_range / 2.0;
    focal_position_[1] = y.minCoeff() + y_range / 2.0;
    focal_position_[2] = z.minCoeff() + z_range / 2.0;

    // create a grid
    vtkSmartPointer<vtkRectilinearGrid> rectilinear_grid =
        vtkSmartPointer<vtkRectilinearGrid>::New();

    vtkSmartPointer<vtkFloatArray> xcoord = vtkSmartPointer<vtkFloatArray>::New();
    xcoord->SetNumberOfComponents(1);
    xcoord->SetNumberOfTuples(size_x);
    vtkSmartPointer<vtkFloatArray> ycoord = vtkSmartPointer<vtkFloatArray>::New();
    ycoord->SetNumberOfComponents(1);
    ycoord->SetNumberOfTuples(size_y);

    for (int i = 0; i < size_x; i++)
      xcoord->InsertComponent(i, 0, x(i));
    for (int i = 0; i < size_y; i++)
      ycoord->InsertComponent(i, 0, y(i));

    // Specify the dimensions of the grid
    rectilinear_grid->SetDimensions(size_x, size_y, 1);
    rectilinear_grid->SetXCoordinates(xcoord);
    rectilinear_grid->SetYCoordinates(ycoord);

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
    rectilinear_grid->GetPointData()->SetScalars(colors);

    return rectilinear_grid;
  }
};
}

#endif /* CONTOUR_PLOT_HPP */
