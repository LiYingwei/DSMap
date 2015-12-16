#-------------------------------------------------
#
# Project created by QtCreator 2015-11-28T15:27:19
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = LiMap
CONFIG   += console
CONFIG   += c++11
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp \
    ywmap.cpp \
    Libs/pugixml/pugixml.cpp \
    load.cpp \
    ploymethods.cpp \
    plot.cpp \
    shortestpath.cpp \
    sa.cpp \
    cmd.cpp \
    nearest.cpp \
    visitprivateelement.cpp \
    query.cpp \
    taxi.cpp

INCLUDEPATH += /usr/local/include/boost /usr/local/include/opencv2 /usr/local/include/

LIBS += -L/usr/local/lib -lopencv_core -lopencv_highgui -lopencv_imgproc

LIBS += -lboost_system-mt #-lboost_filesystem-mt

HEADERS += \
    ywmap.h \
    Libs/pugixml/pugiconfig.hpp \
    Libs/pugixml/pugixml.hpp

QMAKE_MAC_SDK = macosx10.11

QMAKE_CXXFLAGS += -O2

QMAKE_CXXFLAGS += -Wno-unused-parameter
QMAKE_CXXFLAGS += -Wno-unused-variable -Wno-unused-local-typedef

