#-------------------------------------------------
#
# Project created by QtCreator 2011-06-29T19:07:52
#
#-------------------------------------------------

QT       += core gui opengl xml

TARGET = ModelViewer
TEMPLATE = app

config += CONSOLE

win32{
    #DEBUG
    LIBS += C:/Windows/SysWOW64/QGLViewerd2.dll
    #RELEASE
    #LIBS += C:/Windows/System32/QGLViewer2.dll

    INCLUDEPATH += ../include/GL
}

unix{
LIBS += -L../lib \
        -lQGLViewer
}

INCLUDEPATH += ../include/QGLViewer \
               ../include/vcglib \
               ../include \
               ../Testers/Cages \
               ../AirLib \
               ../Airlib/render \
			   ../Airlib/Computation \
			   ../Airlib/DataStructures \
			   ../Airlib/utils \
			   ../Airlib/ui
			   
SOURCES += main.cpp\
        mainwindow.cpp \
        GLWidget.cpp \
        manipulatedFrameSetConstraint.cpp \
        DrawObject.cpp \
        treemodel.cpp \
        treeitem.cpp \
        bar.cpp \
        ../AirLib/DataStructures/Modelo.cpp \
        ../AirLib/DataStructures/Scene.cpp \
        ../AirLib/DataStructures/Geometry.cpp \
        ../AirLib/DataStructures/Deformer.cpp \
		../AirLib/DataStructures/grid3D.cpp \
        ../AirLib/DataStructures/Object.cpp \
        ../AirLib/DataStructures/DataStructures.cpp \
        ../AirLib/DataStructures/Cage.cpp \
        ../AirLib/DataStructures/Node.cpp \
        ../AirLib/DataStructures/skeleton.cpp \
        ../AirLib/DataStructures/InteriorDistancesData.cpp \
        ../AirLib/Computation/MeanValueCoords.cpp \
        ../AirLib/Computation//HarmonicCoords.cpp \
		../AirLib/Computation/GreenCoords.cpp \
        ../AirLib/Computation/mvc_interiorDistances.cpp \
        ../AirLib/Computation/Segmentation.cpp \		
        ../AirLib/ui/outliner.cpp \
        ../AirLib/ui/selectionManager.cpp \		
        ../AirLib/render/gridRender.cpp \
        ../AirLib/render/shadingNode.cpp \
		../AirLib/render/geometryRender.cpp \		
		../AirLib/utils/util.cpp \
		../AirLib/utils/utilQT.cpp \
        ../AirLib/utils/ioWeights.cpp


HEADERS  += mainwindow.h \
			GLWidget.h \
			manipulatedFrameSetConstraint.h \
			DrawObject.h \
			treemodel.h \
			treeitem.h \
			bar.h \
			../AirLib/DataStructures/Modelo.h\
			../AirLib/DataStructures/Scene.h \
			../AirLib/DataStructures/Geometry.h \
			../AirLib/DataStructures/Deformer.h \
			../AirLib/DataStructures/grid3D.h \
			../AirLib/DataStructures/Object.h \
			../AirLib/DataStructures/DataStructures.h \
			../AirLib/DataStructures/Cage.h \
			../AirLib/DataStructures/Node.h \
			../AirLib/DataStructures/skeleton.h \
			../AirLib/DataStructures/InteriorDistancesData.h \		
			../AirLib/Computation/MeanValueCoords.h \
			../AirLib/Computation//HarmonicCoords.h \
			../AirLib/Computation/GreenCoords.h \
			../AirLib/Computation/mvc_interiorDistances.h \
			../AirLib/Computation/Segmentation.h \			
			../AirLib/ui/outliner.h \
			../AirLib/ui/selectionManager.h \			
			../AirLib/render/gridRender.h \
			../AirLib/render/shadingNode.h \
			../AirLib/render/geometryRender.h \			
			../AirLib/utils/util.h \
			../AirLib/utils/utilGL.h \
			../AirLib/utils/utilQT.h \
			../AirLib/utils/ioWeights.h \	
			../AirLib/global.h

FORMS    += mainwindow.ui

RESOURCES += \
    modelViewerResources.qrc


