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

using namespace cv;
using namespace srcl_ctrl;
using namespace std;
using namespace octomap;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
	vtk_viewer_(new VtkViewer(parent)),
	map_viewer_(new MapViewer())
{
    ui->setupUi(this);

    // 2d visualization
    image_label_ = new ImageLabel(this);
    ui->tab2DScene->layout()->addWidget(image_label_);
    ui->tab2DScene->layout()->update();

    //ui->actionOpenMap->setIcon(QIcon(":/icons/icons/open_map.ico"));

    ui->rbUseSGrid->setChecked(true);
    ui->sbQTreeMaxDepth->setValue(6);
    ui->sbQTreeMaxDepth->setMinimum(0);

    // 3d visualization - vtk
    //ui->tab3DScene->layout()->addWidget(qvtk_widget_);
    ui->tab3DScene->layout()->addWidget(vtk_viewer_->GetQVTKWidget());
    ui->tab3DScene->layout()->update();

    vtk_viewer_->ResetView();

    decompose_config_.method = CellDecompMethod::QUAD_TREE;

    // connect image label with main window
//    connect(image_label_,SIGNAL(NewImagePositionClicked(long, long, double)),this,SLOT(UpdateTargetPosition(long, long, double)));
//    connect(ui->btnSendTraj, SIGNAL (clicked()), this, SLOT (BtnSendTrajectory()));
}

MainWindow::~MainWindow()
{
    delete ui;
    delete image_label_;
    delete vtk_viewer_;
    delete map_viewer_;
}

void MainWindow::UpdateDisplayMap()
{
    if(map_viewer_->GetLoadedMap().data)
    {
        Mat vis_img = map_viewer_->DecomposeWorkspace(decompose_config_);

        QImage map_image = ConvertMatToQImage(vis_img);
        QPixmap pix = QPixmap::fromImage(map_image);

        image_label_->setPixmap(pix);
        image_label_->update();
    }
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

void srcl_ctrl::MainWindow::on_actionOpenMap_triggered()
{
    QString map_file_name = QFileDialog::getOpenFileName(this,
        tr("Open Map File"), "/home/rdu/Workspace/srcl_robot_suite/srcl_ctrl/pc/planning/data", tr("Map Images (*.png *.jpg)"));

    bool read_result = map_viewer_->ReadMapFromFile(map_file_name.toStdString());

    if(read_result)
    {
    	std::string stdmsg = "Successfully loaded map: " + map_file_name.toStdString();
    	QString msg = QString::fromStdString(stdmsg);
    	ui->statusBar->showMessage(msg);

    	// set the map tab to be active
    	ui->tabWidget->setCurrentIndex(ui->tabWidget->indexOf(ui->tab2DScene));

    	this->UpdateDisplayMap();
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

    this->UpdateDisplayMap();

    std::cout << "seleted quad tree" << std::endl;
}

void srcl_ctrl::MainWindow::on_rbUseSGrid_clicked()
{
	decompose_config_.method = CellDecompMethod::SQUARE_GRID;

    ui->sbQTreeMaxDepth->setEnabled(false);
    ui->lbQTreeMaxDepth->setEnabled(false);

    this->UpdateDisplayMap();
    std::cout << "selected square grid" << std::endl;
}

void srcl_ctrl::MainWindow::on_sbQTreeMaxDepth_valueChanged(int val)
{
	decompose_config_.method = CellDecompMethod::QUAD_TREE;
	decompose_config_.qtree_depth = val;

    this->UpdateDisplayMap();
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
//    	OcTree* tree = new octomap::OcTree(octomap_filename);

//    	std::cout << "octree size: " << tree->size() << std::endl;
//        if (!raw_image_.data) {
//            printf("No image data \n");
//            ui->statusBar->showMessage(tr("Failed to load map."));

//            return;
//        }
//        else {
//            this->UpdateDisplayMap(raw_image_);

//            std::string stdmsg = "Successfully loaded map: " + map_file_name.toStdString();
//            QString msg = QString::fromStdString(stdmsg);
//            ui->statusBar->showMessage(msg);
//        }
    }
}

void srcl_ctrl::MainWindow::on_pushButton_clicked()
{
	map_viewer_->SaveResultToFile("test_result");
}
