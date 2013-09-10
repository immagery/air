#include "object.h"

using namespace vcg;

object::object() : node()
{
    pos = Point3d(0,0,0);
    rot = Point3d(0,0,0);

    shading = new shadingNode();

    loadIdentity();

    dirtyFlag = false;
}

object::object(unsigned int id) : node(id)
{
    pos = Point3d(0,0,0);
    rot = Point3d(0,0,0);

    shading = new shadingNode(id+1);

    loadIdentity();

    dirtyFlag = false;
}

object::object(vcg::Point3d _pos) : node()
{
    pos = Point3d(_pos);
    rot = Point3d(0,0,0);

    shading = new shadingNode();

    loadIdentity();

    dirtyFlag = false;
}

object::object(vcg::Point3d _pos, vcg::Point3d _rot) : node()
{
    pos = Point3d(_pos);
    rot = Point3d(_rot);

    shading = new shadingNode();

    loadIdentity();

    dirtyFlag = false;
}

void object::resetTransformation()
{
    pos = Point3d(0,0,0);
    rot = Point3d(0,0,0);
    dirtyFlag = true;
}

void object::addTranslation(double tx, double ty, double tz)
{
    // Aplicar la rotación, creo que hay que hacerlo con una multiplicacion.
    pos += Point3d(tx, ty, tz);
    dirtyFlag = true;
}

void object::addRotation(double rx, double ry, double rz)
{
    // Aplicar la rotación, creo que hay que hacerlo con una multiplicacion.
    rot += Point3d(rx, ry, rz);
    dirtyFlag = true;
}

bool object::propagateDirtyness()
{
    dirtyFlag = false;
    return true;
}

void object::loadIdentity()
{
    for(int i = 0; i< 16; i++)
    {
        if(i % 5 == 0)
            tMatrix[i] = 1;
        else
            tMatrix[i] = 0;
    }
}

void object::select(bool bToogle, unsigned int id)
{
    if(shading)
        shading->selected = bToogle;
}

bool object::update()
{
	if(shading)
		return shading->update(this);

	return true;
}
