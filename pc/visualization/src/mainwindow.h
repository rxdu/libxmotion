/*
 * main_window.cpp
 *
 *  Created on: Jul 26, 2016
 *      Author: rdu
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// C++ standard headers
#include <vector>
#include <memory>
#include <cstdint>

// Qt headers
#include <QMainWindow>
#include <QMouseEvent>

// OpenCV headers
#include "opencv2/opencv.hpp"

// Octomap headers
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

// User headers
#include "image_label.h"
#include "vtk_viewer.h"
#include "map_viewer.h"

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
    VtkViewer* vtk_viewer_;
    MapViewer* map_viewer_;

private:
    // workspace decomposition
    DecomposeConfig decompose_config_;

private:
    void SetupMap();
    cv::Mat DecomposeWorkspace(cv::Mat map_img, CellDecompMethod method);
    void UpdateDisplayMap();

//    void UpdateMap();

private:
    QImage ConvertMatToQImage(const cv::Mat& mat);
//    void UpdateQuadPositionOnMap(const geometry_msgs::PoseStampedConstPtr& msg);

//public slots:
//	void UpdateTargetPosition(long x, long y, double raw2scale_ratio);
//	void BtnSendTrajectory();

private slots:
    void on_rbUseQTree_clicked();
    void on_rbUseSGrid_clicked();
    void on_sbQTreeMaxDepth_valueChanged(int val);
    void on_pushButton_clicked();

    void on_actionOpenMap_triggered();
    void on_actionResetView_triggered();
    void on_actionFullView_triggered();
    void on_actionOpenOctomap_triggered();

};
}

#endif // MAINWINDOW_H
