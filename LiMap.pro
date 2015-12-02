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
    plot.cpp

INCLUDEPATH += /usr/local/include

LIBS += \
        -L/usr/local/lib -lopencv_calib3d -lopencv_contrib -lopencv_core\
         -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui\
         -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_nonfree -lopencv_objdetect\
         -lopencv_ocl -lopencv_photo -lopencv_stitching -lopencv_superres\
         -lopencv_ts -lopencv_video -lopencv_videostab

LIBS += -lboost_system-mt -lboost_filesystem-mt

HEADERS += \
    ywmap.h \
    Libs/pugixml/pugiconfig.hpp \
    Libs/pugixml/pugixml.hpp

QMAKE_MAC_SDK = macosx10.11
