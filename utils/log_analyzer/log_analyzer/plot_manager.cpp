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

void PlotManager::PlotData(std::vector<double>& x_data, std::vector<double> y_data, QCPLOT_ID id)
{
//    QVector<double> x(101), y(101); // initialize with entries 0..100
//    for (int i=0; i<101; ++i)
//    {
//        x[i] = i/50.0 - 1; // x goes from -1 to 1
//        y[i] = x[i]*x[i]; // let's plot a quadratic function
//    }

    // check size
    unsigned long size_x = x_data.size();
    unsigned long size_y = y_data.size();
    unsigned long plot_size;

    if(size_x > size_y)
        plot_size = size_y;
    else
        plot_size = size_x;

    // copy data
    QVector<double> x(plot_size), y(plot_size);
    double max_y = 0, min_y = 0;
    for(unsigned long i = 0; i < plot_size; i++)
    {
        x[i] = x_data[i];
        y[i] = y_data[i]/1000;

        if(max_y < y[i])
            max_y = y[i];
        if(min_y > y[i])
            min_y = y[i];
    }

    QCustomPlot* qcplot = GetCustomPlotPtr(id);

    // create graph and assign data to it:
    qcplot->addGraph();
    qcplot->graph(0)->setData(x, y);
    // give the axes some labels:
//    qcplot->xAxis->setLabel("x");
//    qcplot->yAxis->setLabel("y");
    // set axes ranges, so we see all data:
    qcplot->xAxis->setRange(0, x_data[plot_size-1]);
    qcplot->yAxis->setRange(min_y, max_y);
    qcplot->replot();
}
