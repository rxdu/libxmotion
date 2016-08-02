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
#include <string>

// Qt headers
#include <QMainWindow>
#include <QMouseEvent>

// OpenCV headers
#include "opencv2/opencv.hpp"

// User headers
#include "image_label.h"
#include "vtk_viewer.h"
#include "map_viewer.h"

#include "map/map_info.h"
#include "map/map_config.h"
#include "planner/quad_planner.h"

namespace Ui {
class MainWindow;
}

namespace srcl_ctrl {

typedef struct
{
	uint32_t x;
	uint32_t y;
} MapCooridnate;

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
    bool show_padded_area_;
    MapInfo map_info_;

    // planner
    QuadPlanner planner_;
    MapConfig map_config_;
    bool planner_ready_;
    bool use_local_planner_;
    bool start_specified_;
    bool goal_specified_;

private:
    void UpdateWorkspaceMap();
    void ColorCellOnMap(uint32_t x, uint32_t y);
    void DisplayPathOnMap(std::vector<uint64_t>& path);
    MapCooridnate CoordinatesFromDisplayToPadded(long x, long y, double raw2scale_ratio);
    void UpdateGraphPlannerConfig();

public slots:
	void UpdateClickedPosition(long x, long y, double raw2scale_ratio, uint8_t btn_flag);
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

    void on_rbLocalPlanner_clicked();
    void on_rbRemotePlanner_clicked();
    void on_sbSGridCellSize_valueChanged(int val);
    void on_tabWidget_currentChanged(int index);
};
}

#endif // MAINWINDOW_H
