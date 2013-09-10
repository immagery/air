#include <QtGui/qimage.h>
#include <QtGui/qpainter.h>
#include <QtCore/QDir>

#include "bar.h"
#include "utils/util.h"

colorBar::colorBar(QWidget *parent) : QWidget (parent) {
    qimage = new QImage;

    qimage->load(":/modelViewerGeneral/icons/barra_color.jpg");//

    for(int i = 0; i< qimage->width(); i++)
    {
        float colRel = (float)i/(float)qimage->width();
		float r,g,b;
        GetColour(colRel,0,1,r,g,b);
		QColor colValue(r,g,b);
        for(int j = 0; j< qimage->height(); j++)
        {
            qimage->setPixel(i,j, colValue.rgb());
        }
    }
}

void colorBar::paintEvent(QPaintEvent*) {
        QPainter qpainter(this);
        qpainter.drawImage(0, 0, *qimage);
}
