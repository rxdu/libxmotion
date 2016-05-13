#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMouseEvent>

#include <vector>
#include <memory>

#include "opencv2/opencv.hpp"

#include "graph/graph.h"
#include "square_grid/square_grid.h"
#include "gui/image_label.h"

namespace Ui {
class MainWindow;
}

namespace srcl_ctrl {
class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    ImageLabel* image_label_;

//public slots:
//    void UpdateTargetPosition(long x, long y, double raw2scale_ratio);
//    void BtnSendTrajectory();

private:
    std::shared_ptr<SquareGrid> sgrid_;
    std::shared_ptr<Graph<SquareCell>> sgrid_graph_;

    Vertex<SquareCell>* start_sgvertex_;
    Vertex<SquareCell>* end_sgvertex_;
    std::vector<Vertex<SquareCell>*> sg_path_;
//    std::vector<srcl_msgs::UAVTrajectoryPoint> sg_traj_;

    cv::Mat raw_image_;
    cv::Mat qtree_map_;
    cv::Mat sgrid_map_;

    bool disp_once;

//    QuadPlanner* planner_;

private:
    void SetupMap();
//    void UpdateMap();

public:
    QImage ConvertMatToQImage(const cv::Mat& mat);
//    void UpdateQuadPositionOnMap(const geometry_msgs::PoseStampedConstPtr& msg);
private slots:
    void on_actionOpenMap_triggered();
};
}

#endif // MAINWINDOW_H
