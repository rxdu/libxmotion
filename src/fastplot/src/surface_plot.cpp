/* 
 * surface_plot.cpp
 * 
 * Created on: Jan 10, 2018 14:15
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "fastplot/surface_plot.hpp"

#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkStructuredGridGeometryFilter.h>
#include <vtkWarpScalar.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkOutlineFilter.h>
#include <vtkAxesActor.h>
#include <vtkCaptionActor2D.h>
#include <vtkProperty.h>
#include <vtkTextProperty.h>
#include <vtkScalarBarActor.h>

using namespace librav;

SurfacePlot::SurfacePlot()
{
    structured_grid_ = vtkSmartPointer<vtkStructuredGrid>::New();
    renderer_ = vtkSmartPointer<vtkRenderer>::New();
    render_window_ = vtkSmartPointer<vtkRenderWindow>::New();
    render_window_->AddRenderer(renderer_);
    render_window_interactor_ = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    render_window_interactor_->SetRenderWindow(render_window_);
}

// template <typename DerivedVector1, typename DerivedVector2, typename DerivatedMatrix>
// void SurfacePlot::ShowSurface(const Eigen::DenseBase<DerivedVector1> &x, const Eigen::DenseBase<DerivedVector2> &y, const Eigen::DenseBase<DerivatedMatrix> &z)
void SurfacePlot::ShowSurface(const Eigen::VectorXf &x, const Eigen::VectorXf &y, const Eigen::MatrixXf &z)
{
    // Get size of the surface
    const int size_x = x.rows();
    const int size_y = y.rows();

    std::cout << "size_x : " << size_x << std::endl;
    std::cout << "size_y : " << size_y << std::endl;
    std::cout << "size_z : " << z.rows() << " by " << z.cols() << std::endl;

    assert(size_x == z.rows());
    assert(size_y == z.cols());

    structured_grid_->SetDimensions(size_x, size_y, 1);

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    points->SetNumberOfPoints(size_x * size_y);
    for (int j = 0; j < size_y; j++)
        for (int i = 0; i < size_x; i++)
            points->InsertNextPoint(x(i), y(j), z(i, j));
    structured_grid_->SetPoints(points);

    // get scalar field from z-values
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
    structured_grid_->GetPointData()->SetScalars(colors);

    RenderSurface();
}

void SurfacePlot::RenderSurface()
{
    // filter to geometry primitive
    vtkSmartPointer<vtkStructuredGridGeometryFilter> geometry_filter =
        vtkSmartPointer<vtkStructuredGridGeometryFilter>::New();
    geometry_filter->SetInputData(structured_grid_);
    geometry_filter->Update();

    // warp to fit in box
    vtkSmartPointer<vtkWarpScalar> warp = vtkSmartPointer<vtkWarpScalar>::New();

    bool do_warp = true;
    if (do_warp)
    {
        double scale = 10; //Lxy / Lz;
        warp->SetInputConnection(geometry_filter->GetOutputPort());
        warp->XYPlaneOn();
        warp->SetScaleFactor(scale);
    }

    // map gridfunction
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    if (do_warp)
        mapper->SetInputConnection(warp->GetOutputPort());
    else
        mapper->SetInputConnection(geometry_filter->GetOutputPort());
    double tmp[2];
    structured_grid_->GetScalarRange(tmp);
    mapper->SetScalarRange(tmp[0], tmp[1]);

    // create plot surface actor
    vtkSmartPointer<vtkActor> surfplot = vtkSmartPointer<vtkActor>::New();
    surfplot->SetMapper(mapper);

    // create outline
    vtkSmartPointer<vtkOutlineFilter> outlinefilter = vtkSmartPointer<vtkOutlineFilter>::New();
    if (do_warp)
        outlinefilter->SetInputConnection(warp->GetOutputPort());
    else
        outlinefilter->SetInputConnection(geometry_filter->GetOutputPort());
    vtkSmartPointer<vtkPolyDataMapper> outlineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    outlineMapper->SetInputData(outlinefilter->GetOutput());

    // create actor
    vtkSmartPointer<vtkActor> outline = vtkSmartPointer<vtkActor>::New();
    outline->SetMapper(outlineMapper);
    outline->GetProperty()->SetColor(0, 0, 0);

    // create axes
    vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
    axes->SetShaftTypeToCylinder();
    axes->SetNormalizedShaftLength(0.85, 0.85, 0.85);
    axes->SetNormalizedTipLength(0.15, 0.15, 0.15);
    axes->SetCylinderRadius(0.500 * axes->GetCylinderRadius());
    axes->SetConeRadius(1.025 * axes->GetConeRadius());
    axes->SetSphereRadius(1.500 * axes->GetSphereRadius());

    vtkSmartPointer<vtkTextProperty> text_prop_ax = vtkSmartPointer<vtkTextProperty>(axes->GetXAxisCaptionActor2D()->GetCaptionTextProperty());
    text_prop_ax->SetColor(0.0, 0.0, 0.0);
    text_prop_ax->SetFontFamilyToArial();
    text_prop_ax->SetFontSize(8);
    axes->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->ShallowCopy(text_prop_ax);
    axes->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->ShallowCopy(text_prop_ax);

    // create colorbar
    vtkSmartPointer<vtkScalarBarActor> colorbar = vtkSmartPointer<vtkScalarBarActor>::New();
    colorbar->SetLookupTable(mapper->GetLookupTable());
    colorbar->SetWidth(0.085);
    colorbar->SetHeight(0.9);
    colorbar->SetPosition(0.9, 0.1);
    vtkSmartPointer<vtkTextProperty> text_prop_cb = vtkSmartPointer<vtkTextProperty>(colorbar->GetLabelTextProperty());
    text_prop_cb->SetColor(1.0, 1.0, 1.0);
    colorbar->SetLabelTextProperty(text_prop_cb);

    // renderer
    renderer_->AddActor(surfplot);
    bool draw_box = true;
    bool draw_axes = true;
    bool draw_colorbar = true;
    if (draw_box)
        renderer_->AddActor(outline);
    if (draw_axes)
        renderer_->AddActor(axes);
    if (draw_colorbar)
        renderer_->AddActor(colorbar);

    renderer_->SetBackground(0.25, 0.25, 0.25);

    render_window_->Render();
    render_window_interactor_->Start();
}