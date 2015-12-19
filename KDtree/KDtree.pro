#-------------------------------------------------
#
# Project created by QtCreator 2015-12-18T19:41:25
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = KDtree
CONFIG   += console
CONFIG   -= app_bundle
CONFIG   += c++11

TEMPLATE = app


SOURCES += main.cpp \
    kdtree.cpp

HEADERS += \
    kdtree.h

QMAKE_CXXFLAGS += -O2
