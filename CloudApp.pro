QT       += core gui serialport network opengl openglwidgets concurrent

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17


CONFIG -= depend_includepath

DEFINES += WELD_SEARCH_STATIC

INCLUDEPATH += $$PWD/include
LIBS += -L$$PWD/lib \
    -lcloud_selve \
    -lRobotSDK

include($$PWD/qmake/GalaxySDK.pri)
include($$PWD/qmake/pcl1.12.1.pri)
include($$PWD/qmake/opencv.pri)
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
    opencvset.cpp \
    src/imagewriter.cpp \
    src/weld_search.cpp

HEADERS += \
    delectseam.h \
    opencvset.h \
    include/imageframe.h \
    include/imagewriter.h \
    include/weld_search.h \
    include/weld_search_global.h

FORMS += \
    delectseam.ui \
    opencvset.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

win32 {
    CONFIG(release, debug|release) {
        DEPLOY_DIR = $$OUT_PWD/release
    } else {
        DEPLOY_DIR = $$OUT_PWD/debug
    }

    QMAKE_POST_LINK += $$quote(cmd /c copy /Y "$$shell_path($$PWD/cloud_selve.dll)" "$$shell_path($$DEPLOY_DIR)") $$escape_expand(\n\t)
    QMAKE_POST_LINK += $$quote(cmd /c copy /Y "$$shell_path($$PWD/RobotSDK.dll)" "$$shell_path($$DEPLOY_DIR)") $$escape_expand(\n\t)
}
