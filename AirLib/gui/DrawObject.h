#ifndef OBJECT_H_
#define OBJECT_H_

#include "QGLViewer/frame.h"
#include <DataStructures/Object.h>

class DrawObject : object
{
public :
  void draw() const;
  qglviewer::Frame frame;
  int id;
};

#endif // OBJECT_H_
