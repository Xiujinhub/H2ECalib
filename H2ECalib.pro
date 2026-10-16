TEMPLATE = app
CONFIG += console c++17
CONFIG -= app_bundle
CONFIG -= qt

HEADERS += \
    calib/BoardImageCalibrator.h \
    calib/CalibrationTypes.h \
    calib/CoordinateTransformer.h \
    calib/IntrinsicCalibrator.h \
    calib/Math.h \
    calib/Pose3d.h \
    calib/RobotCameraCalibrator3D.h

SOURCES += \
        calib/BoardImageCalibrator.cpp \
        calib/CoordinateTransformer.cpp \
        calib/IntrinsicCalibrator.cpp \
        calib/Math.cpp \
        calib/RobotCameraCalibrator3D.cpp \
        main.cpp

#opencv4.10.0
win32: LIBS += -L$$PWD/SDK/opencv4100/lib/ -lopencv_world4100

INCLUDEPATH += $$PWD/SDK/opencv4100/include
DEPENDPATH += $$PWD/SDK/opencv4100/include

# Eigen
INCLUDEPATH += $$PWD/SDK/Eigen/eigen3
