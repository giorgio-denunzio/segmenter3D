#-------------------------------------------------
#
# Project created by QtCreator 2014-05-01T14:24:33
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 5): QT += widgets

TARGET = segmenter
#TARGET = pcl_visualizer
TEMPLATE = app


SOURCES += main.cpp\
        pclviewer.cpp \
    CSegmenterRegionGrowing.cpp \
    pgmb_io.c \
    CSegmenterColumns.cpp

HEADERS  += pclviewer.h \
    CSegmenterRegionGrowing.h \
    pgmb_io.h \
    CSegmenterColumns.h

FORMS    += pclviewer.ui
