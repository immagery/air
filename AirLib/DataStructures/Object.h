#ifndef OBJECT_H
#define OBJECT_H

#include <vcg/complex/complex.h>
#include "Node.h"
#include <render/ShadingNode.h>

using namespace std;
using namespace vcg;

// Each element of the scene
// Contains data of the transformation
// and operations to handle transformations.
class object : public node
{

public:
    Point3d pos;
    Point3d rot;

    double tMatrix[16];

    shadingNode* shading;

    object();
    object(unsigned int id);
    object(vcg::Point3d _pos);
    object(vcg::Point3d _pos, vcg::Point3d _rot);

    // transformation functions
    void resetTransformation();
    virtual void addTranslation(double tx, double ty, double tz);
    virtual void addRotation(double rx, double ry, double rz);

    // node info propagation specification
    virtual bool update();
    virtual bool propagateDirtyness();

    void loadIdentity();

    // Selection Functions.
    virtual void select(bool bToogle, unsigned int id);
    virtual Point3d getSelCenter(){ return pos; }

    virtual bool getBoundingBox(Point3d& minAuxPt,Point3d& maxAuxPt){ minAuxPt = minAuxPt; maxAuxPt = maxAuxPt; return false;}

};

#endif // OBJECT_H
