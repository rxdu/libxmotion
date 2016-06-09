#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// C++ standard headers
#include <vector>
#include <memory>
#include <cstdint>

// Qt headers
#include <QMainWindow>
#include <QMouseEvent>

// VTK headers
#include <vtkSmartPointer.h>
#include <vtkProperty.h>

#include <QVTKWidget.h>
#include <vtkOrientationMarkerWidget.h>

#include <vtkSphereSource.h>
#include <vtkCubeSource.h>
#include <vtkPlaneSource.h>

#include <vtkDataSetMapper.h>
#include <vtkPolyDataMapper.h>

#include <vtkAxesActor.h>

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

// OpenCV headers
#include "opencv2/opencv.hpp"

// User headers
#include "graph/graph.h"
#include "square_grid/square_grid.h"
#include "quadtree/quad_tree.h"

#include "image_label.h"

namespace Ui {
class MainWindow;
}

namespace srcl_ctrl {

enum class CellDecompMethod {
    SQUARE_GRID,
    QUAD_TREE
};

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    ImageLabel* image_label_;

    QVTKWidget* qvtk_widget_;
    vtkSmartPointer<vtkRenderer> vtk_renderer_;
    vtkSmartPointer<vtkAxesActor> ori_axes_actor_;
    vtkSmartPointer<vtkAxesActor> world_axes_actor_;
    vtkSmartPointer<vtkCamera> vtk_camera_;
    vtkSmartPointer<vtkOrientationMarkerWidget> vtk_ori_marker_widget_;

private:
    // workspace decomposition
    CellDecompMethod cell_decomp_method_;
    std::shared_ptr<SquareGrid> sgrid_;
    std::shared_ptr<Graph<SquareCell>> sgrid_graph_;

    std::shared_ptr<QuadTree> qtree_;
    std::shared_ptr<Graph<QuadTreeNode>> qtree_graph_;
    uint8_t qtree_depth_;

    // workspace search
    Vertex<SquareCell>* start_sgvertex_;
    Vertex<SquareCell>* end_sgvertex_;

    std::vector<Vertex<SquareCell>*> sg_path_;
    std::vector<Vertex<QuadTreeNode>*> qt_path_;

    cv::Mat raw_image_;
    cv::Mat qtree_map_;
    cv::Mat sgrid_map_;

    bool disp_once;
//    QuadPlanner* planner_;

private:
    void SetupMap();
    cv::Mat DecomposeWorkspace(cv::Mat map_img, CellDecompMethod method);
    void UpdateDisplayMap(cv::Mat map_image);

    void InitVTKView();
    void ResetView();
//    void UpdateMap();

private:
    QImage ConvertMatToQImage(const cv::Mat& mat);
//    void UpdateQuadPositionOnMap(const geometry_msgs::PoseStampedConstPtr& msg);

//public slots:
//	void UpdateTargetPosition(long x, long y, double raw2scale_ratio);
//	void BtnSendTrajectory();

private slots:
    void on_actionOpenMap_triggered();
    void on_actionResetView_triggered();
    void on_rbUseQTree_clicked();
    void on_rbUseSGrid_clicked();
    void on_sbQTreeMaxDepth_valueChanged(int val);
    void on_actionFullView_triggered();
};
}

#endif // MAINWINDOW_H
