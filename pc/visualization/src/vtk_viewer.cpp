/*
 * vtk_viewer.cpp
 *
 *  Created on: Jun 8, 2016
 *      Author: rdu
 */

#include "vtk_viewer.h"

using namespace srcl_ctrl;

VtkViewer::VtkViewer(QWidget* parent):
		qvtk_widget_(new QVTKWidget(parent))
{
	// Add shape to display for debugging
	vtkSmartPointer<vtkSphereSource> sphereSource =
			vtkSmartPointer<vtkSphereSource>::New();
	sphereSource->Update();
	vtkSmartPointer<vtkPolyDataMapper> sphereMapper =
			vtkSmartPointer<vtkPolyDataMapper>::New();
	sphereMapper->SetInputConnection(sphereSource->GetOutputPort());
	vtkSmartPointer<vtkActor> sphereActor =
			vtkSmartPointer<vtkActor>::New();
	sphereActor->SetMapper(sphereMapper);

	vtkSmartPointer<vtkCubeSource> cubeSource =
			vtkSmartPointer<vtkCubeSource>::New();
	cubeSource->Update();
	vtkSmartPointer<vtkPolyDataMapper> cubeMapper =
			vtkSmartPointer<vtkPolyDataMapper>::New();
	cubeMapper->SetInputConnection(cubeSource->GetOutputPort());
	vtkSmartPointer<vtkActor> cubeActor =
			vtkSmartPointer<vtkActor>::New();
	cubeActor->SetMapper(cubeMapper);

	// VTK Renderer
	vtkSmartPointer<vtkRenderer> renderer =
			vtkSmartPointer<vtkRenderer>::New();
	renderer->GradientBackgroundOn();
	renderer->SetBackground(0.0, 0.0, 0.0);
	renderer->SetBackground2(0.1, 0.2, 0.4);
	//renderer->AddActor(sphereActor);
	renderer->AddActor(cubeActor);

	// VTK/Qt
	qvtk_widget_->GetRenderWindow()->AddRenderer(renderer);

	// Set up the orientation widget
	qvtk_widget_->GetRenderWindow()->GetInteractor()->Disable();

	vtkSmartPointer<vtkAxesActor> ori_axes =
			vtkSmartPointer<vtkAxesActor>::New();
	ori_axes->SetXAxisLabelText("x");
	ori_axes->SetYAxisLabelText("y");
	ori_axes->SetZAxisLabelText("z");

	vtkSmartPointer<vtkOrientationMarkerWidget> widget =
			vtkSmartPointer<vtkOrientationMarkerWidget>::New();
	widget->SetOutlineColor( 0.9300, 0.5700, 0.1300 );
	widget->SetOrientationMarker( ori_axes );
	widget->SetInteractor(qvtk_widget_->GetRenderWindow()->GetInteractor());
	widget->SetViewport( 0.0, 0.0, 0.2, 0.2 );
	widget->SetEnabled( 1 );
	widget->InteractiveOff();

	qvtk_widget_->GetRenderWindow()->GetInteractor()->Enable();

	renderer->ResetCamera();
}

VtkViewer::~VtkViewer()
{

}
