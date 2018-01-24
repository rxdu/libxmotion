/* 
 * contour_plot.cpp
 * 
 * Created on: Jan 23, 2018 17:08
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "fastplot/contour_plot.hpp"

#include <vtkDataSetMapper.h>
#include <vtkPolyDataMapper.h>
#include <vtkOutlineFilter.h>
#include <vtkRectilinearGridGeometryFilter.h>
#include <vtkContourFilter.h>
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

ContourPlot::ContourPlot()
{
    renderer_ = vtkSmartPointer<vtkRenderer>::New();
    render_window_ = vtkSmartPointer<vtkRenderWindow>::New();
    render_window_interactor_ = vtkSmartPointer<vtkRenderWindowInteractor>::New();

    renderer_->GetActiveCamera()->SetViewUp(0, 0, 1);
    renderer_->GetActiveCamera()->SetPosition(camera_position_);
    renderer_->GetActiveCamera()->SetFocalPoint(focal_position_);
}

void ContourPlot::SetCameraPosition(double x, double y, double z)
{
    camera_position_[0] = x;
    camera_position_[1] = y;
    camera_position_[2] = z;
}

void ContourPlot::SetFocalPosition(double x, double y, double z)
{
    focal_position_[0] = x;
    focal_position_[1] = y;
    focal_position_[2] = z;
}

void ContourPlot::RenderContour(vtkSmartPointer<vtkRectilinearGrid> rectilinear_grid, bool show_surf, int line_num, bool show_box, bool show_axes, bool show_bar)
{
    // create a new renderer
    renderer_ = vtkSmartPointer<vtkRenderer>::New();
    // renderer_->GetActiveCamera()->SetViewUp(0, 0, 1);
    renderer_->GetActiveCamera()->SetPosition(camera_position_);
    renderer_->GetActiveCamera()->SetFocalPoint(focal_position_);

    /**************************** Setup the contour ****************************/
    vtkSmartPointer<vtkRectilinearGridGeometryFilter> geometryFilter =
        vtkSmartPointer<vtkRectilinearGridGeometryFilter>::New();
    geometryFilter->SetInputData(rectilinear_grid);
    geometryFilter->Update();

    // create a poly data mapper and surf actor
    double grid_scalar_range[2];
    vtkSmartPointer<vtkPolyDataMapper> polyMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    polyMapper->SetInputConnection(geometryFilter->GetOutputPort());
    rectilinear_grid->GetScalarRange(grid_scalar_range);
    polyMapper->SetScalarRange(grid_scalar_range[0], grid_scalar_range[1]);
    vtkSmartPointer<vtkActor> surfActor = vtkSmartPointer<vtkActor>::New();
    surfActor->SetMapper(polyMapper);

    vtkSmartPointer<vtkContourFilter> contourFilter = vtkSmartPointer<vtkContourFilter>::New();
    contourFilter->SetInputConnection(geometryFilter->GetOutputPort());
    double tempdiff = (grid_scalar_range[1] - grid_scalar_range[0]) / (10 * line_num);
    contourFilter->GenerateValues(line_num, grid_scalar_range[0] + tempdiff, grid_scalar_range[1] - tempdiff);
    vtkSmartPointer<vtkPolyDataMapper> contourMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    contourMapper->SetInputConnection(contourFilter->GetOutputPort());
    if (show_surf)
        contourMapper->ScalarVisibilityOff();
    else
        contourMapper->SetScalarRange(grid_scalar_range[0], grid_scalar_range[1]);
    vtkSmartPointer<vtkActor> contourActor = vtkSmartPointer<vtkActor>::New();
    contourActor->SetMapper(contourMapper);

    /**************************** Setup the outline ****************************/
    // add outline to the surface
    vtkSmartPointer<vtkOutlineFilter> outlineFilter = vtkSmartPointer<vtkOutlineFilter>::New();
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
    if (show_surf)
	colorbarActor->SetLookupTable(polyMapper->GetLookupTable());
    else
    colorbarActor->SetLookupTable(contourMapper->GetLookupTable());
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
    renderer_->AddActor(contourActor);
    if (show_surf)
	    renderer_->AddActor(surfActor);
    if (show_box)
        renderer_->AddActor(outlineActor);
    if (show_axes)
        renderer_->AddActor(axesActor);
    if (show_bar)
        renderer_->AddActor(colorbarActor);
    renderer_->SetBackground(.2, .3, .4);

    renderer_->ResetCamera();
}

void ContourPlot::ShowRenderToWindow()
{
    // setup renderer, render window, and interactor
    render_window_->AddRenderer(renderer_);

    // Render and interact
    render_window_->Render();
}

void ContourPlot::ShowRenderToWindowWithInteraction()
{
    // setup renderer, render window, and interactor
    render_window_->AddRenderer(renderer_);
    render_window_interactor_->SetRenderWindow(render_window_);

    // Render and interact
    render_window_->Render();
    render_window_interactor_->Start();
}

void ContourPlot::SaveRenderToFile(std::string file_name, int32_t pixel_x, int32_t pixel_y)
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