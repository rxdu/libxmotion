#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QIcon>
#include <QAction>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    QPixmap openico("/icon/open_file.png");
    ui->mainToolBar->addAction(QIcon("icon/open_file.ico"), "Open File");
}

MainWindow::~MainWindow()
{
    delete ui;
}
