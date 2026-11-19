OPENCV_DIR = "D:/opencv/opencv4110/build"

INCLUDEPATH += "$$OPENCV_DIR/include"

CONFIG(release, debug|release) {
    LIBS += -L"$$OPENCV_DIR/x64/vc16/lib" -lopencv_world4110
} else {
    LIBS += -L"$$OPENCV_DIR/x64/vc16/lib" -lopencv_world4110d
}
