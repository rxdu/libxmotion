#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QIcon>
#include <QAction>
#include <QList>

#include <iostream>
#include <sstream>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    plot_manager_(new PlotManager(parent))
{
    ui->setupUi(this);

//    QPixmap openico("/icon/open_file.png");
    QIcon openfile_ico("/icons/open_file.ico");
    ui->mainToolBar->addAction(openfile_ico, "Open File");

//    /* setup central widget layout */
//    // add qcustomplot widgets to plot log data
//    QGridLayout *central_widget_layout = new QGridLayout;
//    int plot_col_span = 17;
//    central_widget_layout->addWidget(plot_manager_->GetCustomPlotPtr(QCPLOT_ID::QCPLOT0),0,0,1,plot_col_span);
//    central_widget_layout->addWidget(plot_manager_->GetCustomPlotPtr(QCPLOT_ID::QCPLOT1),1,0,1,plot_col_span);
//    central_widget_layout->addWidget(plot_manager_->GetCustomPlotPtr(QCPLOT_ID::QCPLOT2),2,0,1,plot_col_span);
//    central_widget_layout->addWidget(plot_manager_->GetCustomPlotPtr(QCPLOT_ID::QCPLOT3),3,0,1,plot_col_span);

//    // add a vertical separation line
//    sep_vline1 = new QFrame(this);
//    sep_vline1->setObjectName(QString::fromUtf8("line"));
//    sep_vline1->setFrameShape(QFrame::VLine);
//    sep_vline1->setFrameShadow(QFrame::Sunken);
//    central_widget_layout->addWidget(sep_vline1, 0, plot_col_span, 4, 1);

//    // configure plots
////    plot_config_group = new QGroupBox(tr("Select Active Plot"));

////    plot_config_rbtns[0] = new QRadioButton(tr("Plot 1"));
////    plot_config_rbtns[1] = new QRadioButton(tr("Plot 2"));
////    plot_config_rbtns[2] = new QRadioButton(tr("Plot 3"));
////    plot_config_rbtns[3] = new QRadioButton(tr("Plot 4"));
////    plot_config_rbtns[0]->setChecked(true);
////    plot_cofig_vbox = new QVBoxLayout;
////    for(int i = 0; i < 4; i++)
////        plot_cofig_vbox->addWidget(plot_config_rbtns[i]);
////    plot_cofig_vbox->addStretch(1);
////    plot_config_group->setLayout(plot_cofig_vbox);
////    central_widget_layout->addWidget(plot_config_group, 0, plot_col_span+1, 1, 2);

//    // add tree view for log head
//    logheadview = new QTreeView();
//    central_widget_layout->addWidget(logheadview, 2, plot_col_span+1, 2, 2);
//    logheadview->setContextMenuPolicy(Qt::CustomContextMenu);
//    loghead_stditem_model = new QStandardItemModel();

//    ui->centralWidget->setLayout(central_widget_layout);

//    // configure slots and signals
//    ConfigGuiEvents();
}

MainWindow::~MainWindow()
{
    // delete log variables
    delete plot_manager_;

    // delete ui elements
    delete logheadview;
    loghead_stditem_model->clear();
    delete loghead_stditem_model;
    delete sep_vline1;
    delete plot_config_group;
    for(int i = 0; i < 4; i++)
        delete plot_config_rbtns[i];

    delete ui;
}

void MainWindow::ConfigGuiEvents()
{
//    connect(log_head_view_,&QTreeWidget::customContextMenuRequested,this,&MainWindow::on_treeWidget_customContextMenuRequested);
}

void MainWindow::on_actionExit_triggered()
{
    this->close();
}

/***************************************************************************************************/
/*------------------------------------------ Log Related ------------------------------------------*/
/***************************************************************************************************/

void MainWindow::on_actionOpenLogFile_triggered()
{
    //get a filename to open
    QString fileName = QFileDialog::getOpenFileName(this,
         tr("Open Log File"), "/home/rdu", tr("Log Files (*.log)"));

    if(!fileName.isEmpty()) {
        std::ostringstream strstream;
        strstream << "Load log file: " << fileName.toStdString() << std::endl;
        log_parser_.ParseLogFile(fileName.toStdString());
        ui->statusBar->showMessage(QString::fromStdString(strstream.str()));

        UpdateLogDataHeads();
//        std::vector<double> x,y;
        plot_manager_->PlotData(log_parser_.log_data_[0],log_parser_.log_data_[1],QCPLOT_ID::QCPLOT0);
        plot_manager_->PlotData(log_parser_.log_data_[0],log_parser_.log_data_[2],QCPLOT_ID::QCPLOT1);
        plot_manager_->PlotData(log_parser_.log_data_[0],log_parser_.log_data_[3],QCPLOT_ID::QCPLOT2);
        plot_manager_->PlotData(log_parser_.log_data_[0],log_parser_.log_data_[4],QCPLOT_ID::QCPLOT3);
    }
}

void MainWindow::UpdateLogDataHeads()
{
    loghead_stditem_model->clear();
    QStandardItem *rootNode = loghead_stditem_model->invisibleRootItem();

    // add items into the tree view
    QList<QStandardItem *> items;
    for(auto it = log_parser_.log_head_.begin(); it != log_parser_.log_head_.end(); it++) {
        QStandardItem* item = new QStandardItem(QString::fromStdString(*it));
        item->setEditable(false);
        items.push_back(item);
    }

    //building up the hierarchy
    rootNode->appendRows(items);

    //register the model
    logheadview->setModel(loghead_stditem_model);
    logheadview->expandAll();
}
