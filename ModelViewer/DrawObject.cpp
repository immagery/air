#include <qgl.h>
#ifdef WIN32
#include <GL/glu.h>
#endif
#include "DrawObject.h"

using namespace qglviewer;

void DrawObject::draw() const
{
  static GLUquadric* quad = gluNewQuadric();

  glPushMatrix();
  glMultMatrixd(frame.matrix());
  gluSphere(quad, 0.03, 10, 6);
  gluCylinder(quad, 0.03, 0.0, 0.09, 10, 1);
  glPopMatrix();
}
