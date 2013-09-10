#ifndef MANIPULATEDFRAMESETCONSTRAINT_H
#define MANIPULATEDFRAMESETCONSTRAINT_H

#include "QGLViewer/constraint.h"
#include "DrawObject.h"

#include "DataStructures/DataStructures.h"

class ManipulatedFrameSetConstraint : public qglviewer::Constraint
{
public:
  ManipulatedFrameSetConstraint();
  void clearSet();
  void addObjectToSet(DrawObject* o);

  virtual void constrainTranslation(qglviewer::Vec &translation, qglviewer::Frame *const frame);
  virtual void constrainRotation(qglviewer::Quaternion &rotation, qglviewer::Frame *const frame);

private :

  QList<DrawObject*> objects_;

public:
  MyMesh* newCage;
  bool transformed;

};

#endif // MANIPULATEDFRAMESETCONSTRAINT_H
