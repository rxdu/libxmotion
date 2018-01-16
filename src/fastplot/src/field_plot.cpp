/* 
 * field_plot.cpp
 * 
 * Created on: Jan 16, 2018 13:06
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "fastplot/field_plot.hpp"

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

FieldPlot::FieldPlot(int64_t size_x, int64_t size_y) : field_size_x_(size_x),
                                                       field_size_y_(size_y)
{
}

void FieldPlot::RenderField(vtkSmartPointer<vtkStructuredGrid> structured_grid, bool do_warp, double wrap_scale, bool show_box, bool show_axes, bool show_bar)
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
    warp->SetScaleFactor(wrap_scale);

    // create a grid mapper and actor
    vtkSmartPointer<vtkDataSetMapper> gridMapper = vtkSmartPointer<vtkDataSetMapper>::New();
    vtkSmartPointer<vtkActor> gridActor = vtkSmartPointer<vtkActor>::New();
    if (do_warp)
        gridMapper->SetInputConnection(warp->GetOutputPort());
    else
        gridMapper->SetInputConnection(geometryFilter->GetOutputPort());
    gridMapper->SetScalarRange(0, 1);
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
