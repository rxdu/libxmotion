/* 
 * surface_plot.cpp
 * 
 * Created on: Jan 10, 2018 14:15
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "fastplot/surface_plot.hpp"

#include <vtkDataSetMapper.h>
#include <vtkPolyDataMapper.h>
#include <vtkOutlineFilter.h>
#include <vtkStructuredGridGeometryFilter.h>
#include <vtkStructuredGridOutlineFilter.h>
#include <vtkWarpScalar.h>
#include <vtkProperty.h>
#include <vtkTextProperty.h>
#include <vtkCamera.h>
#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkCaptionActor2D.h>
#include <vtkTextActor.h>
#include <vtkScalarBarActor.h>
#include <vtkActorCollection.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>

using namespace librav;

SurfacePlot::SurfacePlot()
{
    // structured_grid_ = vtkSmartPointer<vtkStructuredGrid>::New();
    renderer_ = vtkSmartPointer<vtkRenderer>::New();
    render_window_ = vtkSmartPointer<vtkRenderWindow>::New();
    render_window_interactor_ = vtkSmartPointer<vtkRenderWindowInteractor>::New();

    renderer_->GetActiveCamera()->SetViewUp(0, 0, 1);
    renderer_->GetActiveCamera()->SetPosition(camera_position_);
    renderer_->GetActiveCamera()->SetFocalPoint(focal_position_);
}

void SurfacePlot::SetCameraPosition(double x, double y, double z)
{
    camera_position_[0] = x;
    camera_position_[1] = y;
    camera_position_[2] = z;
}

void SurfacePlot::SetFocalPosition(double x, double y, double z)
{
    focal_position_[0] = x;
    focal_position_[1] = y;
    focal_position_[2] = z;
}

void SurfacePlot::EnableAutoScaleRange(bool enable)
{
    z_auto_scale_ = enable;
}

void SurfacePlot::SetScaleRange(double min, double max)
{
    z_scale_min_ = min;
    z_scale_max_ = max;
    z_auto_scale_ = false;
}

// Final scale factor = std::min(x_range, y_range)/ z_range / wrap_scale_factor_;
// This factor controls how much you want to increase the scale of z values so that
//  it's easier to see the height difference on the field plane.
void SurfacePlot::SetWrapScaleFactor(double scale)
{
    wrap_scale_factor_ = scale;
}

void SurfacePlot::RenderSurface(vtkSmartPointer<vtkStructuredGrid> structured_grid, bool do_warp, double wrap_scale, bool show_box, bool show_axes, bool show_bar)
{
    // create a new renderer
    renderer_ = vtkSmartPointer<vtkRenderer>::New();
    renderer_->GetActiveCamera()->SetViewUp(0, 0, 1);
    renderer_->GetActiveCamera()->SetPosition(camera_position_);
    renderer_->GetActiveCamera()->SetFocalPoint(focal_position_);

    /**************************** Setup the grid ****************************/
    vtkSmartPointer<vtkStructuredGridGeometryFilter> geometryFilter =
        vtkSmartPointer<vtkStructuredGridGeometryFilter>::New();
    geometryFilter->SetInputData(structured_grid);
    geometryFilter->Update();

    // create a warper
    vtkSmartPointer<vtkWarpScalar> warp = vtkSmartPointer<vtkWarpScalar>::New();
    warp->SetInputConnection(geometryFilter->GetOutputPort());
    warp->XYPlaneOn();
    warp->SetScaleFactor(wrap_scale / wrap_scale_factor_);

    // create a grid mapper and actor
    vtkSmartPointer<vtkDataSetMapper> gridMapper = vtkSmartPointer<vtkDataSetMapper>::New();
    vtkSmartPointer<vtkActor> gridActor = vtkSmartPointer<vtkActor>::New();
    if (do_warp)
        gridMapper->SetInputConnection(warp->GetOutputPort());
    else
        gridMapper->SetInputConnection(geometryFilter->GetOutputPort());

    if (z_auto_scale_)
    {
        double grid_scalar_range[2];
        structured_grid->GetScalarRange(grid_scalar_range);
        gridMapper->SetScalarRange(grid_scalar_range[0], grid_scalar_range[1]);
    }
    else
    {
        gridMapper->SetScalarRange(z_scale_min_, z_scale_max_);
    }
    gridActor->SetMapper(gridMapper);
    gridActor->GetProperty()->EdgeVisibilityOn();
    // gridActor->GetProperty()->SetEdgeColor(0, 0, 1);

    /**************************** Setup the outline ****************************/
    // add outline to the surface
    vtkSmartPointer<vtkOutlineFilter> outlineFilter = vtkSmartPointer<vtkOutlineFilter>::New();
    if (do_warp)
        outlineFilter->SetInputData(warp->GetOutput());
    else
        outlineFilter->SetInputData(geometryFilter->GetOutput());
    outlineFilter->Update();
    vtkSmartPointer<vtkPolyDataMapper> outlineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    outlineMapper->SetInputConnection(outlineFilter->GetOutputPort());

    // create outline actor
    vtkSmartPointer<vtkActor> outlineActor = vtkSmartPointer<vtkActor>::New();
    outlineActor->SetMapper(outlineMapper);
    outlineActor->GetProperty()->SetColor(0, 0, 0);

    /**************************** Setup the axes ****************************/
    vtkSmartPointer<vtkAxesActor> axesActor = vtkSmartPointer<vtkAxesActor>::New();
    // axesActor->SetShaftTypeToLine();
    axesActor->GetXAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
    axesActor->GetYAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
    axesActor->GetZAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();

    vtkSmartPointer<vtkTextProperty> text_prop_ax = vtkSmartPointer<vtkTextProperty>(axesActor->GetXAxisCaptionActor2D()->GetCaptionTextProperty());
    text_prop_ax->SetColor(0.0, 0.0, 0.0);
    text_prop_ax->SetFontFamilyToArial();
    text_prop_ax->SetFontSize(20);
    // text_prop_ax->SetOpacity(0.35);
    axesActor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->ShallowCopy(text_prop_ax);
    axesActor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->ShallowCopy(text_prop_ax);

    /**************************** Setup the colorbar ****************************/
    // create colorbar
    vtkSmartPointer<vtkScalarBarActor> colorbarActor = vtkSmartPointer<vtkScalarBarActor>::New();
    colorbarActor->SetLookupTable(gridMapper->GetLookupTable());
    colorbarActor->SetWidth(0.065);
    colorbarActor->SetHeight(0.2);
    colorbarActor->SetPosition(0.9, 0.1);
    vtkSmartPointer<vtkTextProperty> text_prop_cb = vtkSmartPointer<vtkTextProperty>(colorbarActor->GetLabelTextProperty());
    text_prop_cb->SetColor(1.0, 1.0, 1.0);
    colorbarActor->SetLabelTextProperty(text_prop_cb);

    /**************************** Setup the renderer ****************************/
    // Remove existing actors
    if (renderer_->GetActors() != nullptr)
    {
        vtkProp *next = renderer_->GetActors()->GetNextActor();
        while (next != nullptr)
        {
            renderer_->RemoveActor(next);
            next = renderer_->GetActors()->GetNextActor();
        }
    }

    // Add the actor to the scene
    renderer_->AddActor(gridActor);
    if (show_box)
        renderer_->AddActor(outlineActor);
    if (show_axes)
        renderer_->AddActor(axesActor);
    if (show_bar)
        renderer_->AddActor(colorbarActor);
    renderer_->SetBackground(.2, .3, .4);

    renderer_->ResetCamera();
}

void SurfacePlot::ShowRenderToWindow()
{
    // setup renderer, render window, and interactor
    render_window_->AddRenderer(renderer_);

    // Render and interact
    render_window_->Render();
}

void SurfacePlot::ShowRenderToWindowWithInteraction()
{
    // setup renderer, render window, and interactor
    render_window_->AddRenderer(renderer_);
    render_window_interactor_->SetRenderWindow(render_window_);

    // Render and interact
    render_window_->Render();
    render_window_interactor_->Start();
}

void SurfacePlot::SaveRenderToFile(std::string file_name, int32_t pixel_x, int32_t pixel_y)
{
    // setup renderer, render window, and interactor
    render_window_->SetSize(pixel_x, pixel_y);
    render_window_->AddRenderer(renderer_);
    render_window_->Render();

    // save to file
    vtkSmartPointer<vtkWindowToImageFilter> w2i = vtkSmartPointer<vtkWindowToImageFilter>::New();
    vtkSmartPointer<vtkPNGWriter> pngfile = vtkSmartPointer<vtkPNGWriter>::New();

    w2i->SetInput(render_window_);
    w2i->Update();

    pngfile->SetInputConnection(w2i->GetOutputPort());
    pngfile->SetFileName(file_name.c_str());

    pngfile->Write();
}