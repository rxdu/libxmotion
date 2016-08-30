/*
 * vtk_viewer.cpp
 *
 *  Created on: Jun 8, 2016
 *      Author: rdu
 */

#include <vtkCamera.h>

#include "vtk_viewer.h"

#include <vtkPNGReader.h>
#include <vtkImageActor.h>
#include <vtkImageMapper.h>
#include <vtkImageSliceMapper.h>
#include <vtkImageSlice.h>

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
	cubeSource->SetXLength(1.5);
	cubeSource->SetYLength(2.5);
	cubeSource->Update();
	vtkSmartPointer<vtkPolyDataMapper> cubeMapper =
			vtkSmartPointer<vtkPolyDataMapper>::New();
	cubeMapper->SetInputConnection(cubeSource->GetOutputPort());
	vtkSmartPointer<vtkActor> cubeActor =
			vtkSmartPointer<vtkActor>::New();
	cubeActor->SetMapper(cubeMapper);
	cubeActor->GetProperty()->SetColor(0.8,0.2,0.3);

	// Create a plane
	vtkSmartPointer<vtkPlaneSource> planeSource =
			vtkSmartPointer<vtkPlaneSource>::New();
	double gridSize = 10.0;
	double scale = 1.0;
	double origin[3],normal[3];
	int cellSize = 1.0;

	origin[0] = 0.0;
	origin[1] = 0.0;
	origin[2] = 0.0;

	normal[0] = 0.0;
	normal[1] = 0.0;
	normal[2] = 1.0;

	planeSource->SetOrigin(-gridSize*scale, -gridSize*scale, 0.0);
	planeSource->SetPoint1(gridSize*scale, -gridSize*scale, 0.0);
	planeSource->SetPoint2(-gridSize*scale, gridSize*scale, 0.0);
	planeSource->SetResolution(gridSize/cellSize, gridSize/cellSize);
	planeSource->SetCenter(origin);
	planeSource->SetNormal(normal);
	planeSource->Update();

	vtkPolyData* plane = planeSource->GetOutput();

	// Create a mapper and actor
	vtkSmartPointer<vtkPolyDataMapper> plane_mapper =
			vtkSmartPointer<vtkPolyDataMapper>::New();
	plane_mapper->SetInputData(plane);

	vtkSmartPointer<vtkActor> plane_actor =
			vtkSmartPointer<vtkActor>::New();
	plane_actor->GetProperty()->SetRepresentationToWireframe();
	plane_actor->SetMapper(plane_mapper);

	// World frame
	world_axes_actor_ = vtkSmartPointer<vtkAxesActor>::New();
	world_axes_actor_->SetOrigin(0.0,0.0,0.0);
	world_axes_actor_->SetXAxisLabelText("wx");
	world_axes_actor_->SetYAxisLabelText("wy");
	world_axes_actor_->SetZAxisLabelText("wz");
	world_axes_actor_->AxisLabelsOff ();

	// Camera
	vtk_camera_ = vtkSmartPointer<vtkCamera>::New();

	// VTK Renderer
	vtk_renderer_ = vtkSmartPointer<vtkRenderer>::New();
	vtk_renderer_->GradientBackgroundOn();
	vtk_renderer_->SetBackground(0.0, 0.0, 0.0);
	vtk_renderer_->SetBackground2(0.1, 0.2, 0.4);
	vtk_renderer_->SetActiveCamera(vtk_camera_);
	vtk_renderer_->ResetCamera();

	//	vtk_renderer_->AddActor(sphereActor);
	vtk_renderer_->AddActor(cubeActor);
	vtk_renderer_->AddActor(plane_actor);
	vtk_renderer_->AddActor(world_axes_actor_);

	// Connect the VTK renderer and the QT widget
	qvtk_widget_->GetRenderWindow()->AddRenderer(vtk_renderer_);

	// Set up the orientation widget
	qvtk_widget_->GetRenderWindow()->GetInteractor()->Disable();

	ori_axes_actor_ = vtkSmartPointer<vtkAxesActor>::New();
	ori_axes_actor_->SetXAxisLabelText("x");
	ori_axes_actor_->SetYAxisLabelText("y");
	ori_axes_actor_->SetZAxisLabelText("z");

	vtk_ori_marker_widget_ = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
	vtk_ori_marker_widget_->SetOutlineColor( 0.9300, 0.5700, 0.1300 );
	vtk_ori_marker_widget_->SetOrientationMarker( ori_axes_actor_ );
	vtk_ori_marker_widget_->SetInteractor(qvtk_widget_->GetRenderWindow()->GetInteractor());
	vtk_ori_marker_widget_->SetViewport( 0.0, 0.0, 0.2, 0.2 );
	vtk_ori_marker_widget_->SetEnabled( 1 );
	vtk_ori_marker_widget_->InteractiveOff();

	qvtk_widget_->GetRenderWindow()->GetInteractor()->Enable();

	qvtk_widget_->update();

	DisplayMap();
}

VtkViewer::~VtkViewer()
{
	delete qvtk_widget_;
}

void VtkViewer::ResetView()
{
	//set the camera position and orientation
	vtk_renderer_->GetActiveCamera()->SetFocalPoint(0, 0, 0.0);
	vtk_renderer_->GetActiveCamera()->SetViewUp(0, 0, 1);
	vtk_renderer_->GetActiveCamera()->SetPosition(12.0, 12.0, 5 );

	vtk_renderer_->ResetCamera();

	qvtk_widget_->update();
}

void VtkViewer::ResetCamera()
{
	vtk_renderer_->ResetCamera();

	qvtk_widget_->update();
}

void VtkViewer::UpdateViewer()
{
	qvtk_widget_->update();
}

void VtkViewer::DisplayMap()
{
	// Read the image
	vtkSmartPointer<vtkPNGReader> reader =
			vtkSmartPointer<vtkPNGReader>::New();
	reader->SetFileName("/home/rdu/Workspace/srcl_rtk/srcl_ctrl/pc/planning/data/path_repair_case1.png");

	vtkSmartPointer<vtkImageSliceMapper> imageSliceMapper = vtkSmartPointer<vtkImageSliceMapper>::New();
	//imageSliceMapper->SetInputData(reader.);
	imageSliceMapper->SetInputConnection(reader->GetOutputPort());

	vtkSmartPointer<vtkImageSlice> imageSlice = vtkSmartPointer<vtkImageSlice>::New();
	imageSlice->SetMapper(imageSliceMapper);

	vtk_renderer_->AddViewProp(imageSlice);

	qvtk_widget_->update();
}
