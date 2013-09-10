#ifndef __BAR_H__
#define __BAR_H__

#include <qwidget.h>

class QImage;

class colorBar : public QWidget {
public:
        colorBar(QWidget *parent);

protected:
        void paintEvent(QPaintEvent*);

private:
        QImage *qimage;
};

#endif
