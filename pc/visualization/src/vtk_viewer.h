/*
 * vtk_viewer.h
 *
 *  Created on: Jun 8, 2016
 *      Author: rdu
 */

#ifndef VISUALIZATION_SRC_VTK_VIEWER_H_
#define VISUALIZATION_SRC_VTK_VIEWER_H_

#include <QWidget>
#include <QVTKWidget.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSphereSource.h>
#include <vtkCubeSource.h>
#include <vtkPlaneSource.h>
#include <vtkSmartPointer.h>
#include <vtkProperty.h>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkRenderWindowInteractor.h>

namespace srcl_ctrl {

class VtkViewer
{
public:
	VtkViewer(QWidget* parent=0);
	~VtkViewer();

private:
	QVTKWidget* qvtk_widget_;
	vtkSmartPointer<vtkRenderer> vtk_renderer_;
	vtkSmartPointer<vtkAxesActor> ori_axes_actor_;
	vtkSmartPointer<vtkAxesActor> world_axes_actor_;
	vtkSmartPointer<vtkCamera> vtk_camera_;
	vtkSmartPointer<vtkOrientationMarkerWidget> vtk_ori_marker_widget_;

public:
	QVTKWidget* GetQVTKWidget(){return qvtk_widget_;};

	void ResetView();
	void ResetCamera();
	void UpdateViewer();
};

}


#endif /* VISUALIZATION_SRC_VTK_VIEWER_H_ */
