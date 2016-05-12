/*
 * quad_gui.cpp
 *
 *  Created on: Apr 26, 2016
 *      Author: rdu
 */

#include "mainwindow.h"
#include <QApplication>
#include <ros/ros.h>

using namespace srcl_ctrl;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    MainWindow w;
    w.show();

    return a.exec();
}


