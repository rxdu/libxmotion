/*
 * vtk_viewer.h
 *
 *  Created on: Jun 8, 2016
 *      Author: rdu
 */

#ifndef GUI_SRC_VTK_VIEWER_H_
#define GUI_SRC_VTK_VIEWER_H_

#include <QWidget>
#include <QVTKWidget.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSphereSource.h>
#include <vtkCubeSource.h>
#include <vtkSmartPointer.h>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkRenderWindowInteractor.h>

namespace srcl_ctrl {

class VtkViewer
{
public:
	VtkViewer(QWidget* parent=0);
	~VtkViewer();

public:
	QVTKWidget* qvtk_widget_;

};

}


#endif /* GUI_SRC_VTK_VIEWER_H_ */
