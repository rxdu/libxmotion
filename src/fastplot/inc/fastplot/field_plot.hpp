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

#include "fastplot/surface_plot.hpp"
#include "fastplot/contour_plot.hpp"

namespace librav
{
struct FieldPlot
{
  FieldPlot()
  {
    // parameters chosen for better field visualization
    surface_plot.SetWrapScaleFactor(4.0);
    surface_plot.SetScaleRange(0.0, 1.0);
  };

  SurfacePlot surface_plot;
  ContourPlot contour_plot;
};

namespace FastPlot
{
// This function could be called in a loop to show the changing process of a surface
template <typename DerivedVector1, typename DerivedVector2, typename DerivatedMatrix>
void ShowFieldSurfaceFrame(const Eigen::MatrixBase<DerivedVector1> &x, const Eigen::MatrixBase<DerivedVector2> &y, const Eigen::MatrixBase<DerivatedMatrix> &z, bool do_warp = false, double cam_pos_x = 0, double cam_pos_y = 50, double cam_pos_z = 50, bool show_box = true, bool show_axes = true, bool show_bar = true)
{
  FieldPlot fplot;
  fplot.surface_plot.SetCameraPosition(cam_pos_x, cam_pos_y, cam_pos_z);
  fplot.surface_plot.ShowSurfaceFrame(x, y, z, do_warp, show_box, show_axes, show_bar);
}

// This function shows the surface with mouse interactions enabled
template <typename DerivedVector1, typename DerivedVector2, typename DerivatedMatrix>
void ShowFieldSurface(const Eigen::MatrixBase<DerivedVector1> &x, const Eigen::MatrixBase<DerivedVector2> &y, const Eigen::MatrixBase<DerivatedMatrix> &z, bool do_warp = false, bool show_box = true, bool show_axes = true, bool show_bar = true)
{
  FieldPlot fplot;
  fplot.surface_plot.ShowSurface(x, y, z, do_warp, show_box, show_axes, show_bar);
}

// This function renders the surface to a file
template <typename DerivedVector1, typename DerivedVector2, typename DerivatedMatrix>
void SaveFieldSurfaceToFile(const Eigen::MatrixBase<DerivedVector1> &x, const Eigen::MatrixBase<DerivedVector2> &y, const Eigen::MatrixBase<DerivatedMatrix> &z, std::string file_name, int32_t pixel_x = 640, int32_t pixel_y = 480, bool do_warp = false, bool show_box = true, bool show_axes = true, bool show_bar = true)
{
  FieldPlot fplot;
  fplot.surface_plot.SaveSurfaceToFile(x, y, z, file_name, pixel_x, pixel_y, do_warp, show_box, show_axes, show_bar);
}

// This function could be called in a loop to show the changing process of a contour
template <typename DerivedVector1, typename DerivedVector2, typename DerivatedMatrix>
void ShowFieldContourFrame(const Eigen::MatrixBase<DerivedVector1> &x, const Eigen::MatrixBase<DerivedVector2> &y, const Eigen::MatrixBase<DerivatedMatrix> &z, bool show_surf = false, bool show_box = true, bool show_axes = true, bool show_bar = true)
{
  FieldPlot fplot;
  fplot.contour_plot.ShowContourFrame(x, y, z, show_surf, show_box, show_axes, show_bar);
}

// This function shows the contour with mouse interactions enabled
template <typename DerivedVector1, typename DerivedVector2, typename DerivatedMatrix>
void ShowFieldContour(const Eigen::MatrixBase<DerivedVector1> &x, const Eigen::MatrixBase<DerivedVector2> &y, const Eigen::MatrixBase<DerivatedMatrix> &z, bool show_surf = false, bool show_box = true, bool show_axes = true, bool show_bar = true)
{
  FieldPlot fplot;
  fplot.contour_plot.ShowContour(x, y, z, show_surf, show_box, show_axes, show_bar);
}

// This function renders the surface to a file
template <typename DerivedVector1, typename DerivedVector2, typename DerivatedMatrix>
void SaveFieldContourToFile(const Eigen::MatrixBase<DerivedVector1> &x, const Eigen::MatrixBase<DerivedVector2> &y, const Eigen::MatrixBase<DerivatedMatrix> &z, std::string file_name, int32_t pixel_x = 640, int32_t pixel_y = 480, bool show_surf = false, bool show_box = true, bool show_axes = true, bool show_bar = true)
{
  FieldPlot fplot;
  fplot.contour_plot.SaveContourToFile(x, y, z, file_name, pixel_x, pixel_y, show_surf, show_box, show_axes, show_bar);
}
}
}

#endif /* FIELD_PLOT_HPP */
