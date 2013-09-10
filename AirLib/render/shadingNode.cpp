#include "ShadingNode.h"

#include <utils/utilGL.h>
#include <DataStructures/Object.h>

bool shadingNode::update(object* obj)
{
	if(!obj) return false;

    if(!obj->dirtyFlag)
        return true;
    else
    {
        glMatrixMode(GL_MODELVIEW_MATRIX);
        glPushMatrix();
        glLoadIdentity();
        glTranslated(obj->pos.X(), obj->pos.Y(), obj->pos.Z());
        glRotated(obj->rot.Z(), 0, 0, 1);
        glRotated(obj->rot.Y(), 0, 1, 0);
        glRotated(obj->rot.X(), 1, 0, 0);
        glGetDoublev(GL_MODELVIEW_MATRIX, obj->tMatrix);
        glPopMatrix();
        dirtyFlag = false;
    }

    return true;
}

void shadingNode::beforeDraw(object* obj)
{
    if(dirtyFlag) 
		update(obj);

    glPushMatrix();
    glMultMatrixd(obj->tMatrix);
}

void shadingNode::afterDraw(object* obj)
{
    glPopMatrix();
}