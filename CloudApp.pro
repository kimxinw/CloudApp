QT       += core gui serialport network opengl openglwidgets concurrent

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17


CONFIG -= depend_includepath

INCLUDEPATH += $$PWD/include
LIBS += -L$$PWD/lib \
    -lcloud_selve \
    -lweld_search \
    -lRobotSDK

include($$PWD/../GalaxySDK.pri)
include($$PWD/../pcl1.12.1.pri)
include($$PWD/../opencv.pri)
INCLUDEPATH += ./eigen-master
INCLUDEPATH += ./json-develop/include
# 为C和C++编译器启用AVX指令集
QMAKE_CXXFLAGS += /arch:AVX

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    delectseam.cpp \
    main.cpp \
    opencvset.cpp

HEADERS += \
    delectseam.h \
    opencvset.h

FORMS += \
    delectseam.ui \
    opencvset.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
