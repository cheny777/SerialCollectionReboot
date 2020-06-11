#-------------------------------------------------
#
# Project created by QtCreator 2020-06-05T10:16:26
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = SerialCollection
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

#串口所需库
QT += serialport

SOURCES += \
        main.cpp \
        widget.cpp \
    drawline.cpp \
    lowpassfilter2p.cpp \
    ritools.cpp \
    kine.cpp \
    robotcfg.cpp \
    PID_v1.cpp

HEADERS += \
        widget.h \
    drawline.h \
    smoothing.h \
    lowpassfilter2p.h \
    ciprotal.h \
    rdef.h \
    ritools.h \
    kine.h \
    robotcfg.h \
    fileportal.h \
    PID_v1.h

FORMS += \
        widget.ui

LIBS += -L$$PWD/./ -lCIPortal
LIBS += -L$$PWD/./ -lFilePortal
