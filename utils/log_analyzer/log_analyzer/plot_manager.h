#ifndef PLOTMANAGER_H
#define PLOTMANAGER_H

#include <vector>
#include <qcustomplot.h>

enum class QCPLOT_ID
{
    QCPLOT0 = 0,
    QCPLOT1,
    QCPLOT2,
    QCPLOT3
};

class PlotManager
{
public:
    PlotManager(QWidget *parent);
    ~PlotManager();

private:
    QWidget *parent_;
    std::vector<QCustomPlot*> qcplots_;

public:
    const int MAX_PLOT_NUM;

public:
    QCustomPlot* GetCustomPlotPtr(QCPLOT_ID id);
};

#endif // PLOTMANAGER_H
