#-------------------------------------------------
#
# Project created by QtCreator 2016-03-11T21:21:44
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = log_analyzer
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    qcustomplot.cpp \
    log_parser.cpp


HEADERS  += mainwindow.h \
    qcustomplot.h \
    log_parser.h

FORMS    += mainwindow.ui

RESOURCES +=
