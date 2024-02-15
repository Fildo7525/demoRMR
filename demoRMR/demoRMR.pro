#-------------------------------------------------
#
# Project created by QtCreator 2018-02-11T14:35:38
#
#-------------------------------------------------

QT       += core gui
CONFIG += c++17

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = demoRMR
TEMPLATE = app
win32 {
LIBS += -lws2_32
LIBS += -lWinmm
}
INCLUDEPATH += ../robot
LIBS += -L../bin -lrobot


SOURCES += main.cpp\
        mainwindow.cpp \
        pidcontroller.cpp \
        robotTrajectoryController.cpp

HEADERS  += mainwindow.h \
    pidcontroller.h \
    robotTrajectoryController.h

FORMS    += mainwindow.ui
