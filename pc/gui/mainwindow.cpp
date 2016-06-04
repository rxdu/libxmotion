#include <common/planning_types.h>
#include <map2d/graph_builder.h>
#include <map2d/qtree_builder.h>
#include <map2d/sgrid_builder.h>
#include <iostream>
#include <cstdint>

#include <QFileDialog>

#include "mainwindow.h"
#include "ui_mainwindow.h"

// user
#include "visualizer/graph_vis.h"

using namespace cv;
using namespace srcl_ctrl;
using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    disp_once(false),
    start_sgvertex_(nullptr),
    end_sgvertex_(nullptr),
    cell_decomp_method_(CellDecompMethod::SQUARE_GRID),
    qtree_depth_(6)
{
    ui->setupUi(this);

    // 2d visualization
    image_label_ = new ImageLabel(this);
    ui->tab2DScene->layout()->addWidget(image_label_);
    ui->tab2DScene->layout()->update();

    ui->actionOpenMap->setIcon(QIcon(":/icons/icons/open_map.ico"));
    ui->rbUseSGrid->setChecked(true);
    ui->sbQTreeMaxDepth->setValue(6);
    ui->sbQTreeMaxDepth->setMinimum(0);

    // 3d visualization - vtk
    qvtk_widget_ = new QVTKWidget;
    ui->tab3DScene->layout()->addWidget(qvtk_widget_);
    ui->tab3DScene->layout()->update();
//    vtk_renderer_ = vtkRenderer::New();
//    vtk_render_win_ = vtkRenderWindow::New();
//    qvtk_widget_->SetRenderWindow(vtk_render_win_);
//    qvtk_widget_->GetRenderWindow()->AddRenderer(vtk_renderer_);
//    vtk_renderer_->SetBackground(0,0,0);
//    vtk_renderer_->Render();

    // connect image label with main window
//    connect(image_label_,SIGNAL(NewImagePositionClicked(long, long, double)),this,SLOT(UpdateTargetPosition(long, long, double)));
//    connect(ui->btnSendTraj, SIGNAL (clicked()), this, SLOT (BtnSendTrajectory()));
}

MainWindow::~MainWindow()
{
    delete ui;
    delete image_label_;
    delete qvtk_widget_;
}

void MainWindow::UpdateDisplayMap(Mat map_img)
{
    if(map_img.data)
    {
        Mat vis_img = this->DecomposeWorkspace(map_img, cell_decomp_method_);

        QImage map_image = ConvertMatToQImage(vis_img);
        QPixmap pix = QPixmap::fromImage(map_image);

        image_label_->setPixmap(pix);
        image_label_->update();
    }
}

Mat MainWindow::DecomposeWorkspace(Mat map_image, CellDecompMethod method)
{
    Mat vis_img;

    std::cout << "this function called" << std::endl;

    if(method == CellDecompMethod::SQUARE_GRID)
    {
        std::tuple<std::shared_ptr<SquareGrid> , Mat> sg_map;

        sg_map = SGridBuilder::BuildSquareGridMap(map_image, 32);
        sgrid_ = std::get<0>(sg_map);
        sgrid_map_ = std::get<1>(sg_map);

        // build graph from square grid
        sgrid_graph_ = GraphBuilder::BuildFromSquareGrid(sgrid_, true);

        GraphVis vis;

        if (sgrid_map_.empty())
            vis.VisSquareGrid(*sgrid_, vis_img);
        else
            vis.VisSquareGrid(*sgrid_, sgrid_map_, vis_img);

        /*** put the graph on top of the square grid ***/
        vis.VisSquareGridGraph(*sgrid_graph_, vis_img, vis_img, true);
        sgrid_map_ = vis_img;

        std::cout << "decomposed using square grid" << std::endl;
    }
    else if(method == CellDecompMethod::QUAD_TREE)
    {
        std::tuple<std::shared_ptr<QuadTree> , Mat> qt_map;

        qt_map = QTreeBuilder::BuildQuadTreeMap(map_image, qtree_depth_);
        qtree_ = std::get<0>(qt_map);
        qtree_map_ = std::get<1>(qt_map);

        // build graph from square grid
        qtree_graph_ = GraphBuilder::BuildFromQuadTree(qtree_);

        GraphVis vis;
        vis.VisQuadTree(*qtree_, qtree_map_, vis_img, TreeVisType::ALL_SPACE);
        vis.VisQTreeGraph(*qtree_graph_, vis_img, vis_img, true, false);

        qtree_map_ = vis_img;

        std::cout << "decomposed using quadtree" << std::endl;
    }

    return vis_img;
}

//void MainWindow::UpdateMap()
//{
//    Mat quad_map;
//    quad_map.create(sgrid_map_.size(), sgrid_map_.type());
//    sgrid_map_.copyTo(quad_map);
//
//    if(start_sgvertex_ != nullptr) {
//        Range rngx(start_sgvertex_->bundled_data_.bbox_.x.min, start_sgvertex_->bundled_data_.bbox_.x.max);
//        Range rngy(start_sgvertex_->bundled_data_.bbox_.y.min, start_sgvertex_->bundled_data_.bbox_.y.max);
//        quad_map(rngy,rngx) = Scalar(28,25,215);
//    }
//
//    if(end_sgvertex_ != nullptr)
//    {
//        Range rngx(end_sgvertex_->bundled_data_.bbox_.x.min, end_sgvertex_->bundled_data_.bbox_.x.max);
//        Range rngy(end_sgvertex_->bundled_data_.bbox_.y.min, end_sgvertex_->bundled_data_.bbox_.y.max);
//        quad_map(rngy,rngx) = Scalar(180,117,69);
//    }
//
//    if(!sg_path_.empty())
//    {
//        GraphVis vis;
//        vis.VisSquareGridPath(sg_path_, quad_map, quad_map);
//    }
//
//    QImage map_image = ConvertMatToQImage(quad_map);
//    QPixmap pix = QPixmap::fromImage(map_image);
//    image_label_->setPixmap(pix);
//    image_label_->update();
//}

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

//void MainWindow::UpdateTargetPosition(long x, long y, double raw2scale_ratio)
//{
//    long x_raw, y_raw;
//    x_raw = x * raw2scale_ratio;
//    y_raw = y * raw2scale_ratio;
//
//    long col = x_raw / sgrid_->cell_size_;
//    long row = y_raw / sgrid_->cell_size_;
//
//    uint32_t id = sgrid_->GetIDFromPosition(row, col);
//
//    end_sgvertex_ = sgrid_graph_->GetVertexFromID(id);
//
//    if(start_sgvertex_ != nullptr && end_sgvertex_ != nullptr) {
//        sg_path_ = sgrid_graph_->AStarSearch(start_sgvertex_, end_sgvertex_);
//
//        std::vector<Position2Dd> waypoints;
//
//        for(auto& vt: sg_path_) {
//            Position2Dd pos;
//            pos.x = vt->bundled_data_.location_.x;
//            pos.y = vt->bundled_data_.location_.y;
//            waypoints.push_back(pos);
//        }
//
//        // prepare conversion variables
//        double world2map_ratio = 25.0/raw_image_.cols;
//        uint64_t padded_x_size = (sgrid_map_.cols - raw_image_.cols)/2;
//        uint64_t padded_y_size = (sgrid_map_.rows - raw_image_.rows)/2;
//
//        // convert waypoints from padded image frame to world frame
//        for(auto& wp : waypoints)
//        {
//            wp.x -= padded_x_size;
//            wp.y -= padded_y_size;
//
//            wp.x *= world2map_ratio;
//            wp.y *= world2map_ratio;
//
//            wp.x = -wp.x + 12.5;
//            wp.y -= 10.0;
//
//            std::cout << "point: " << wp.x << " , " << wp.y << std::endl;
//        }
//
//        sg_traj_.clear();
//        sg_traj_ = planner_->GenerateTrajectory(waypoints);
//    }
//
//    std::cout << "coordinate in scaled image: " << x << " , " << y << std::endl;
//    std::cout << "coordinate in original image: " << x_raw << " , " << y_raw << std::endl;
//    std::cout << "clicked cell id: " << id << std::endl;
//
//    UpdateMap();
//}

//void MainWindow::BtnSendTrajectory()
//{
//    if(!sg_traj_.empty())
//        planner_->SendTrajectory(sg_traj_);
//}

//void MainWindow::UpdateQuadPositionOnMap(const geometry_msgs::PoseStampedConstPtr& msg)
//{
//    double world_x, world_y;
//    world_x = msg->pose.position.x;
//    world_y = msg->pose.position.y;
//
//    // convert to image coordinate
//    double world2map_ratio = 25.0/raw_image_.cols;
//    uint64_t map_x, map_y;
//    map_x = world_x/world2map_ratio;
//    map_y = world_y/world2map_ratio;
//
//    // convert to padded map coordinate
//    uint64_t padded_x_size = (sgrid_map_.cols - raw_image_.cols)/2;
//    uint64_t padded_y_size = (sgrid_map_.rows - raw_image_.rows)/2;
//
//    uint64_t pmap_x, pmap_y;
//    pmap_x = map_x + padded_x_size;
//    pmap_y = map_y + padded_y_size;
//
//
//    // find col and row number in the grid
//    long col = pmap_x / sgrid_->cell_size_;
//    long row = pmap_y / sgrid_->cell_size_;
//
//    uint32_t id = sgrid_->GetIDFromPosition(row, col);
//
//    if(!disp_once){
//        std::cout << "map coordinate: " << map_x << " , " << map_y << std::endl;
//        std::cout << "padded map coordinate: " << pmap_x << " , " << pmap_y << std::endl;
//        std::cout << "id: " << id << std::endl;
//        disp_once = true;
//    }
//
//    start_sgvertex_ = sgrid_graph_->GetVertexFromID(id);
//}

void srcl_ctrl::MainWindow::on_actionOpenMap_triggered()
{
//    std::cout << "clicked" << std::endl;

    QString map_file_name = QFileDialog::getOpenFileName(this,
        tr("Open Map File"), "/home/rdu/Workspace/srcl_robot_suite/srcl_ctrl/planning/data", tr("Map Images (*.png *.jpg)"));

    if(!map_file_name.isEmpty()) {
        // read map image
        raw_image_ = imread(map_file_name.toStdString(), IMREAD_GRAYSCALE);

        if (!raw_image_.data) {
            printf("No image data \n");
            ui->statusBar->showMessage(tr("Failed to load map."));

            return;
        }
        else {
            this->UpdateDisplayMap(raw_image_);

            std::string stdmsg = "Successfully loaded map: " + map_file_name.toStdString();
            QString msg = QString::fromStdString(stdmsg);
            ui->statusBar->showMessage(msg);
        }
    }
}

void srcl_ctrl::MainWindow::on_rbUseQTree_clicked()
{
    cell_decomp_method_ = CellDecompMethod::QUAD_TREE;

    ui->sbQTreeMaxDepth->setEnabled(true);
    ui->lbQTreeMaxDepth->setEnabled(true);

    this->UpdateDisplayMap(raw_image_);

    std::cout << "seleted quad tree" << std::endl;
}

void srcl_ctrl::MainWindow::on_rbUseSGrid_clicked()
{
    cell_decomp_method_ = CellDecompMethod::SQUARE_GRID;

    ui->sbQTreeMaxDepth->setEnabled(false);
    ui->lbQTreeMaxDepth->setEnabled(false);

    this->UpdateDisplayMap(raw_image_);
    std::cout << "selected square grid" << std::endl;
}

void srcl_ctrl::MainWindow::on_sbQTreeMaxDepth_valueChanged(int val)
{
    qtree_depth_ = val;

    this->UpdateDisplayMap(raw_image_);
}
