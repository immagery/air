#-------------------------------------------------
#
# Project created by QtCreator 2012-05-10T19:57:34
#
#-------------------------------------------------

QT       -= gui

QT       += core

TARGET = ArpackPlusLib
TEMPLATE = lib
CONFIG += staticlib

SOURCES += arpackpluslib.cpp

HEADERS += arpackpluslib.h
unix:!symbian {
    maemo5 {
        target.path = /opt/usr/lib
    } else {
        target.path = /usr/lib
    }
    INSTALLS += target
}

macx{
       LIBS += libblas.a \
               libsuperlu.a \
               libarpack.a \
               /usr/local/lib/libf2c.a
}

INCLUDEPATH += ../include/SuperLU \
               ../include/arpack++
