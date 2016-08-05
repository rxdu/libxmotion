/*
 * main_window.cpp
 *
 *  Created on: Jul 26, 2016
 *      Author: rdu
 */

// C++ standard headers
#include <iostream>
#include <cstdint>
#include <cmath>

#include <QFileDialog>

#include "ui_mainwindow.h"

// user
#include "mainwindow.h"
#include "map/map_utils.h"

using namespace cv;
using namespace std;
using namespace srcl_ctrl;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
	image_label_(new ImageLabel(this)),
	vtk_viewer_(new VtkViewer(parent)),
	map_viewer_(new MapViewer()),
	show_padded_area_(true),
	planner_ready_(false),
	use_local_planner_(true),
	start_specified_(false),
	goal_specified_(false)
{
    ui->setupUi(this);
    //ui->actionOpenMap->setIcon(QIcon(":/icons/icons/open_map.ico"));

    // 2d visualization
    ui->tab2DScene->layout()->addWidget(image_label_);
    ui->tab2DScene->layout()->update();

    // 3d visualization - vtk
    ui->tab3DScene->layout()->addWidget(vtk_viewer_->GetQVTKWidget());
    ui->tab3DScene->layout()->update();
    vtk_viewer_->ResetView();

    // default configurations
    ui->rbUseSGrid->setChecked(true);
    ui->sbQTreeMaxDepth->setValue(6);
    ui->sbQTreeMaxDepth->setMinimum(0);
    ui->sbQTreeMaxDepth->setMaximum(8);
    ui->sbQTreeMaxDepth->setEnabled(false);
    ui->lbQTreeMaxDepth->setEnabled(false);
    ui->cbShowPadding->setChecked(true);
    ui->rbLocalPlanner->setChecked(true);
    ui->sbSGridCellSize->setValue(32);
    ui->sbSGridCellSize->setMinimum(16);
    ui->sbSGridCellSize->setMaximum(48);

    decompose_config_.method = CellDecompMethod::SQUARE_GRID;
    decompose_config_.square_cell_size = ui->sbSGridCellSize->value();
    show_padded_area_ = ui->cbShowPadding->isChecked();

    // connect image label with main window
   connect(image_label_,SIGNAL(MapImageClicked(long, long, double, uint8_t)),this,SLOT(UpdateClickedPosition(long, long, double,uint8_t)));
//    connect(ui->btnSendTraj, SIGNAL (clicked()), this, SLOT (BtnSendTrajectory()));
}

MainWindow::~MainWindow()
{
    delete ui;
    delete image_label_;
    delete vtk_viewer_;
    delete map_viewer_;
}

void MainWindow::UpdateWorkspaceMap()
{
	std::cout << "update workspace map" << std::endl;

    if(map_viewer_->HasMapLoaded())
    {
        Mat vis_img = map_viewer_->DecomposeWorkspace(decompose_config_, map_info_);

        if(decompose_config_.method == CellDecompMethod::QUAD_TREE)
        	map_config_.SetMapType(MapDataModel::QUAD_TREE, decompose_config_.qtree_depth);
        else if(decompose_config_.method == CellDecompMethod::SQUARE_GRID)
        	map_config_.SetMapType(MapDataModel::SQUARE_GRID, decompose_config_.square_cell_size);

        this->UpdateGraphPlannerConfig();

        if(!show_padded_area_)
        {
        	Range rngx(0 + map_info_.padded_left, vis_img.cols - map_info_.padded_right);
        	Range rngy(0 + map_info_.padded_top, vis_img.rows - map_info_.padded_bottom);

        	// Points and Size go (x,y); (width,height) ,- Mat has (row,col).
        	vis_img = vis_img(rngy,rngx);
        }

        QImage map_image = ConvertMatToQImage(vis_img);
        QPixmap pix = QPixmap::fromImage(map_image);

        image_label_->setPixmap(pix);
        image_label_->update();
    }
    else
    	std::cerr << "No map loaded for display." << std::endl;
}

void MainWindow::ColorCellOnMap(uint32_t x, uint32_t y)
{
	if(map_viewer_->HasMapLoaded())
	{
		bool updated;
		Mat vis_img = map_viewer_->HighlightSelectedNode(x, y, updated);

		if(updated) {
			if(!show_padded_area_)
			{
				Range rngx(0 + map_info_.padded_left, vis_img.cols - map_info_.padded_right);
				Range rngy(0 + map_info_.padded_top, vis_img.rows - map_info_.padded_bottom);

				// Points and Size go (x,y); (width,height) ,- Mat has (row,col).
				vis_img = vis_img(rngy,rngx);
			}

			QImage map_image = ConvertMatToQImage(vis_img);
			QPixmap pix = QPixmap::fromImage(map_image);

			image_label_->setPixmap(pix);
			image_label_->update();
		}
	}
}

void MainWindow::DisplayPathOnMap(std::vector<uint64_t>& path)
{
	if(map_viewer_->HasMapLoaded())
	{
		Mat vis_img = map_viewer_->DisplayTrajectory(path);

		if(!show_padded_area_)
		{
			Range rngx(0 + map_info_.padded_left, vis_img.cols - map_info_.padded_right);
			Range rngy(0 + map_info_.padded_top, vis_img.rows - map_info_.padded_bottom);

			// Points and Size go (x,y); (width,height) ,- Mat has (row,col).
			vis_img = vis_img(rngy,rngx);
		}

		QImage map_image = ConvertMatToQImage(vis_img);
		QPixmap pix = QPixmap::fromImage(map_image);

		image_label_->setPixmap(pix);
		image_label_->update();
	}
}

MapCooridnate MainWindow::CoordinatesFromDisplayToPadded(long x, long y, double raw2scale_ratio)
{
	MapCooridnate rpos;

	rpos.x = x * raw2scale_ratio;
	rpos.y = y * raw2scale_ratio;

	return rpos;
}

void MainWindow::UpdateGraphPlannerConfig()
{
	planner_.ConfigGraphPlanner(map_config_);
	start_specified_ = false;
	goal_specified_ = false;

	if(planner_.active_graph_planner_ != GraphPlannerType::NOT_SPECIFIED)
		planner_ready_ = true;
	else
		planner_ready_ = false;
}

QImage MainWindow::ConvertMatToQImage(const Mat& mat)
{
    // 8-bits unsigned, NO. OF CHANNELS=1
    if(mat.type()==CV_8UC1)
    {
        // Set the color table (used to translate colour indexes to qRgb values)
        QVector<QRgb> colorTable;
        for (int i=0; i<256; i++)
            colorTable.push_back(qRgb(i,i,i));
        // Copy input Mat
        const uchar *qImageBuffer = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage img(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_Indexed8);
        img.setColorTable(colorTable);
        return img;
    }
    // 8-bits unsigned, NO. OF CHANNELS=3
    if(mat.type()==CV_8UC3)
    {
        // Copy input Mat
        const uchar *qImageBuffer = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage img(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        return img.rgbSwapped();
    }
    else
    {
//        qDebug() << "ERROR: Mat could not be converted to QImage.";
        return QImage();
    }
}

void srcl_ctrl::MainWindow::UpdateClickedPosition(long x, long y, double raw2scale_ratio, uint8_t btn_flag)
{
	std::cout << "mouse clicked position: " << x << " , " << y << std::endl;

	Position2D map_pos;
	MapCooridnate pos = CoordinatesFromDisplayToPadded(x,y,raw2scale_ratio);

	std::cout << "image position: " << pos.x << " , " << pos.y << std::endl;

	if(!show_padded_area_)
	{
		Position2D origin_pos, padded_pos;
		origin_pos.x = pos.x;
		origin_pos.y = pos.y;
		padded_pos = MapUtils::CoordinatesFromOriginalToPadded(origin_pos, map_info_);

//		std::cout << "map info:"
//				<< map_info_.padded_left << " , "
//				<< map_info_.padded_right << " , "
//				<< map_info_.padded_top << " , "
//				<< map_info_.padded_bottom << " , "
//				<< std::endl;
//		std::cout << "padded position: "
//				<< padded_pos.x << " , "
//				<< padded_pos.y << " , "
//				<< std::endl;

		map_pos.x = padded_pos.x;
		map_pos.y = padded_pos.y;
	}
	else
	{
		map_pos.x = pos.x;
		map_pos.y = pos.y;
	}

	ColorCellOnMap(map_pos.x, map_pos.y);

	if(use_local_planner_)
	{
		if(btn_flag & Qt::LeftButton) {
			std::cout << "left button" << std::endl;
			planner_.SetStartMapPosition(map_pos);
			start_specified_ = true;
		}
		else if(btn_flag & Qt::RightButton) {
			std::cout << "right button" << std::endl;
			planner_.SetGoalMapPosition(map_pos);
			goal_specified_ = true;
		}

		if(start_specified_ && goal_specified_ && planner_ready_) {
			auto traj = planner_.SearchForGlobalPath();

			std::cout << "path length: " << traj.size() << std::endl;
			if(!traj.empty())
				DisplayPathOnMap(traj);
		}
	}
	else
	{

	}
}

void srcl_ctrl::MainWindow::on_actionOpenMap_triggered()
{
    QString map_file_name = QFileDialog::getOpenFileName(this,
        tr("Open Map File"), "/home/rdu/Workspace/srcl_robot_suite/srcl_ctrl/pc/planning/data", tr("Map Images (*.png *.jpg)"));

    bool read_result = map_viewer_->ReadMapFromFile(map_file_name.toStdString());

    if(read_result)
    {
    	this->map_config_.SetMapPath(map_file_name.toStdString());

    	std::string stdmsg = "Successfully loaded map: " + map_file_name.toStdString();
    	QString msg = QString::fromStdString(stdmsg);
    	ui->statusBar->showMessage(msg);

    	// set the map tab to be active
    	ui->tabWidget->setCurrentIndex(ui->tabWidget->indexOf(ui->tab2DScene));

    	this->UpdateWorkspaceMap();
    }
    else
    {
    	ui->statusBar->showMessage(tr("Failed to load map."));
    }
}

void srcl_ctrl::MainWindow::on_rbUseQTree_clicked()
{
	decompose_config_.method = CellDecompMethod::QUAD_TREE;

    ui->sbQTreeMaxDepth->setEnabled(true);
    ui->lbQTreeMaxDepth->setEnabled(true);
    ui->sbSGridCellSize->setEnabled(false);
    ui->lbSGridCellSize->setEnabled(false);

    this->UpdateWorkspaceMap();

    std::cout << "seleted quad tree" << std::endl;
}

void srcl_ctrl::MainWindow::on_rbUseSGrid_clicked()
{
	decompose_config_.method = CellDecompMethod::SQUARE_GRID;

	ui->sbSGridCellSize->setEnabled(true);
	ui->lbSGridCellSize->setEnabled(true);
    ui->sbQTreeMaxDepth->setEnabled(false);
    ui->lbQTreeMaxDepth->setEnabled(false);

    this->UpdateWorkspaceMap();
    std::cout << "selected square grid" << std::endl;
}

void srcl_ctrl::MainWindow::on_sbQTreeMaxDepth_valueChanged(int val)
{
	decompose_config_.method = CellDecompMethod::QUAD_TREE;
	decompose_config_.qtree_depth = val;

    this->UpdateWorkspaceMap();
}

void srcl_ctrl::MainWindow::on_sbSGridCellSize_valueChanged(int val)
{
	decompose_config_.method = CellDecompMethod::SQUARE_GRID;
	decompose_config_.square_cell_size = val;

	this->UpdateWorkspaceMap();
}

void srcl_ctrl::MainWindow::on_actionResetView_triggered()
{
	// issue: http://stackoverflow.com/questions/18097521/vtkactor-not-visible-after-render-but-visible-on-camera-resetview
	vtk_viewer_->ResetView();
}

void srcl_ctrl::MainWindow::on_actionFullView_triggered()
{
	vtk_viewer_->ResetCamera();
}

void srcl_ctrl::MainWindow::on_actionOpenOctomap_triggered()
{
    QString octomap_filename = QFileDialog::getOpenFileName(this,
        tr("Open Map File"), "/home/rdu/Workspace/srcl_robot_suite/srcl_ctrl/planning/data", tr("Octomap (*.bt)"));

    if(!octomap_filename.isEmpty()) {
        // read octomap
    }
}

void srcl_ctrl::MainWindow::on_btnSaveMap_clicked()
{
	QString new_filename = QFileDialog::getSaveFileName(this,
				tr("Save Map to File"),"/home/rdu/Workspace/srcl_robot_suite/srcl_ctrl/planning/data",tr("Map Images (*.png *.jpg)"));

	if(!new_filename.contains(".jpg",Qt::CaseSensitivity::CaseInsensitive) &&
			!new_filename.contains(".png",Qt::CaseSensitivity::CaseInsensitive))
	{
		new_filename.append(".jpg");
	}

	map_viewer_->SaveResultToFile(new_filename.toStdString());
}

void srcl_ctrl::MainWindow::on_cbShowPadding_toggled(bool checked)
{
	if(show_padded_area_ != checked)
	{
		show_padded_area_ = checked;
		this->UpdateWorkspaceMap();
	}
}

void srcl_ctrl::MainWindow::on_rbLocalPlanner_clicked()
{
	use_local_planner_ = true;
}

void srcl_ctrl::MainWindow::on_rbRemotePlanner_clicked()
{
	use_local_planner_ = false;
}

void srcl_ctrl::MainWindow::on_tabWidget_currentChanged(int index)
{
    if(index == ui->tabWidget->indexOf(ui->tab2DScene))
        ui->gbWSpace->setVisible(true);
    else
        ui->gbWSpace->setVisible(false);
}
