TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        ArmorFunction.cpp \
        main.cpp

INCLUDEPATH += /usr/local/include \
               /usr/local/include/opencv4

LIBS += /usr/local/lib/*.so \

HEADERS += \
    commen.h

