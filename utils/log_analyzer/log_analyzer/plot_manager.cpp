#include "plot_manager.h"

PlotManager::PlotManager(QWidget *parent):
    parent_(parent),
    MAX_PLOT_NUM(4)
{
    for(int i = 0; i < MAX_PLOT_NUM; i++)
    {
        qcplots_.push_back(new QCustomPlot(parent_));
    }
}

PlotManager::~PlotManager()
{
    for(auto it = qcplots_.begin(); it != qcplots_.end(); it++)
        delete (*it);
    qcplots_.clear();
}

QCustomPlot* PlotManager::GetCustomPlotPtr(QCPLOT_ID id)
{
    switch(id)
    {
    case QCPLOT_ID::QCPLOT0:
        return qcplots_[0];
    case QCPLOT_ID::QCPLOT1:
        return qcplots_[1];
    case QCPLOT_ID::QCPLOT2:
        return qcplots_[2];
    case QCPLOT_ID::QCPLOT3:
        return qcplots_[3];
    default:
        return nullptr;
    }
}
