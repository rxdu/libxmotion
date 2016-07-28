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
    // GUI/Viewer objects
    Ui::MainWindow *ui;
    ImageLabel* image_label_;
    VtkViewer* vtk_viewer_;
    MapViewer* map_viewer_;

private:
    // GUI internal function
    QImage ConvertMatToQImage(const cv::Mat& mat);

private:
    // workspace decomposition
    DecomposeConfig decompose_config_;

private:
    void UpdateDisplayMap();

//public slots:
//	void UpdateTargetPosition(long x, long y, double raw2scale_ratio);
//	void BtnSendTrajectory();

private slots:
    void on_rbUseQTree_clicked();
    void on_rbUseSGrid_clicked();
    void on_sbQTreeMaxDepth_valueChanged(int val);
    void on_cbShowPadding_toggled(bool checked);
    void on_btnSaveMap_clicked();

    void on_actionOpenMap_triggered();
    void on_actionResetView_triggered();
    void on_actionFullView_triggered();
    void on_actionOpenOctomap_triggered();

};
}

#endif // MAINWINDOW_H
