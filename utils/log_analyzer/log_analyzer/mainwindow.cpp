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

    // build menu for items in log head list
    BuildLogHeadMenu();

    /* setup central widget layout */
    // add qcustomplot widgets to plot log data
    QGridLayout *central_widget_layout = new QGridLayout;
    central_widget_layout->addWidget(plot_manager_->GetCustomPlotPtr(QCPLOT_ID::QCPLOT0),0,0,1,4);
    central_widget_layout->addWidget(plot_manager_->GetCustomPlotPtr(QCPLOT_ID::QCPLOT1),1,0,1,4);
    central_widget_layout->addWidget(plot_manager_->GetCustomPlotPtr(QCPLOT_ID::QCPLOT2),2,0,1,4);
    central_widget_layout->addWidget(plot_manager_->GetCustomPlotPtr(QCPLOT_ID::QCPLOT3),3,0,1,4);

    // add a vertical separation line
    sep_vline1 = new QFrame(this);
    sep_vline1->setObjectName(QString::fromUtf8("line"));
    sep_vline1->setFrameShape(QFrame::VLine);
    sep_vline1->setFrameShadow(QFrame::Sunken);
    central_widget_layout->addWidget(sep_vline1, 0, 4, 4, 1);

    // add tree view for log head
    logheadview = new QTreeView();
    central_widget_layout->addWidget(logheadview, 0, 5, 4, 1);
    logheadview->setContextMenuPolicy(Qt::CustomContextMenu);
    loghead_stditem_model = new QStandardItemModel();

    ui->centralWidget->setLayout(central_widget_layout);

    // configure slots and signals
    ConfigGuiEvents();
}

MainWindow::~MainWindow()
{
    // delete log variables
    delete plot_manager_;

    // delete ui elements
    delete logheadview;
    delete logheadmenu;
    loghead_stditem_model->clear();
    delete loghead_stditem_model;
    delete sep_vline1;

    delete ui;
}

void MainWindow::ConfigGuiEvents()
{
//    connect(log_head_view_,&QTreeWidget::customContextMenuRequested,this,&MainWindow::on_treeWidget_customContextMenuRequested);
    connect(logheadview, SIGNAL(customContextMenuRequested(QPoint)),this, SLOT(on_logheadview_customContextMenuRequested(QPoint)));
}

void MainWindow::BuildLogHeadMenu()
{
    logheadmenu = new QMenu(this);
    logheadmenu->addAction(QString("Copy"));
}

void MainWindow::on_logheadview_customContextMenuRequested(const QPoint &pos)
{
//    QMenu menu;

////    menu.addAction(QString("Copy"), this,SLOT(on_copy()));
//    menu.addAction(QString("Copy"));
//    menu.popup(log_head_view_->viewport()->mapToGlobal(pos));
//    QMenu *menu=new QMenu(this);
//    menu->addAction(QString("Copy"), this,SLOT(on_copy()));
    logheadmenu->popup(logheadview->viewport()->mapToGlobal(pos));
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
