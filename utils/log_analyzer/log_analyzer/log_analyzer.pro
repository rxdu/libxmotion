#-------------------------------------------------
#
# Project created by QtCreator 2016-03-11T21:21:44
#
#-------------------------------------------------

QT       += core gui
QMAKE_CXXFLAGS += -std=c++11

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = log_analyzer
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    qcustomplot.cpp \
    log_parser.cpp \
    plot_manager.cpp


HEADERS  += mainwindow.h \
    qcustomplot.h \
    log_parser.h \
    plot_manager.h

FORMS    += mainwindow.ui

RESOURCES +=
