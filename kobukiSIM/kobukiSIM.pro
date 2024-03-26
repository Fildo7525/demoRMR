#-------------------------------------------------
#
# Project created by QtCreator 2018-02-11T14:35:38
#
#-------------------------------------------------

QT       += core gui
QT +=network
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
CONFIG += c++14

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
# OpenCV
win32 {
    INCLUDEPATH += C:/opencv_vc16/include/

    LIBS +=-LC:/opencv_vc16/bin
    LIBS +=-LC:/opencv_vc16/lib

    CONFIG(debug, debug|release) {
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_core440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_highgui440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_imgcodecs440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_imgproc440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_features2d440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_calib3d440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_videoio440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_ml440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_dnn440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_flann440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_objdetect440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_photo440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_shape440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_video440d
    }
    else {
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_core440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_highgui440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_imgcodecs440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_imgproc440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_features2d440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_calib3d440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_videoio440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_ml440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_dnn440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_flann440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_objdetect440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_photo440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_shape440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_video440
    }
}
unix {
QMAKE_LFLAGS += -no-pie
    PKGCONFIG += opencv4

INCLUDEPATH += /usr/include/opencv4/

    LIBS += -L/usr/lib/x86_64-linux-gnu        \
        -l:libopencv_core.so       \
        -l:libopencv_highgui.so    \
        -l:libopencv_imgcodecs.so  \
        -l:libopencv_imgproc.so    \
        -l:libopencv_features2d.so\
        -l:libopencv_calib3d.so    \
        -l:libopencv_videoio.so    \
        -l:libopencv_ml.so         \
        -l:libopencv_dnn.so        \
        -l:libopencv_flann.so      \
        -l:libopencv_objdetect.so \
        -l:libopencv_photo.so      \
        -l:libopencv_video.so
}
TARGET = kobukiSIM
TEMPLATE = app
#LIBS += -lws2_32

SOURCES += main.cpp\
        mainwindow.cpp \
    rplidar.cpp \
    CKobuki.cpp \
    map_loader.cpp \
    pozyxsim.cpp

HEADERS  += mainwindow.h \
    rplidar.h \
    CKobuki.h \
    map_loader.h \
    pozyxsim.h

FORMS    += mainwindow.ui
