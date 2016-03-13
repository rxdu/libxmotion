#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QTreeView>
#include <QStringList>
#include <QStandardItemModel>
#include <QStandardItem>
#include <QGroupBox>
#include <QRadioButton>

#include <qcustomplot.h>

#include "plot_manager.h"
#include "log_parser.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_actionOpenLogFile_triggered();
    void on_actionExit_triggered();

private:
    // UI elements
    Ui::MainWindow *ui;
    QFrame* sep_vline1;
    QTreeView *logheadview;
    QMenu *logheadmenu;
    QStandardItemModel *loghead_stditem_model;
    QGroupBox* plot_config_group;
    QRadioButton* plot_config_rbtns[4];
    QVBoxLayout* plot_cofig_vbox;

    // log related
    PlotManager* plot_manager_;
    LogParser log_parser_;

private:
    void ConfigGuiEvents();
    void UpdateLogDataHeads();
};

#endif // MAINWINDOW_H
