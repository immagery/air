#include "SkeletonRender.h"
#include <utils/utilGL.h>

#define NORMALR 0.5
#define NORMALG 0
#define NORMALB 0.5

#define SELR 0.1
#define SELG 0.8
#define SELB 0.1

#define SUBSELR 1.0
#define SUBSELG 1.0
#define SUBSELB 1.0

// render size
#define DEFAULT_SIZE 0.05

#include <DataStructures/Skeleton.h>

void SkeletonRender::drawFunc(skeleton* obj)
{
    // transformaciones
    beforeDraw(obj);

	/*
	Geometry* geom = (Geometry*)obj;

    // Pintamos en modo directo el modelo
    if(shtype == T_POLY || shtype == T_XRAY)
    {

        float blend = 1;
        if(shtype == T_XRAY)
        {
            blend = 0.3;
            glEnable(GL_BLEND);
            //glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        }

        glBegin(GL_TRIANGLES);

        int size = colors.size();
        if(size<=0)
            glColor4f(color[0]*blend, color[1]*blend, color[2]*blend,blend);

        MyMesh::FaceIterator fi;
        for(fi = geom->face.begin(); fi!= geom->face.end(); ++fi )
         {
             //glNormal3dv(&(*fi).N()[0]);
             for(int i = 0; i<3; i++)
             {
                 int pIdx = fi->V(i)->IMark();

                 if(size > 0 && pIdx < size)
                 {
                     if(colors[pIdx].size()== 3)
                        glColor4f(colors[pIdx][0]*blend, colors[pIdx][1]*blend, colors[pIdx][2]*blend,blend);
                     else
                        glColor4f(color[0]*blend, color[1]*blend, color[2]*blend,blend);
                 }

                 glNormal(fi->V(i)->N());
                 glVertex(fi->V(i)->P());
             }
         }
        glEnd();


        if(shtype == T_XRAY)
        {
            glDisable(GL_BLEND);
            //glBlendFunc(GL_ONE, GL_ONE);
        }
    }
    else if(shtype == T_LINES && !selected)
    {

        glDisable(GL_LIGHTING);
        glColor3f(color[0], color[1], color[2]);
        MyMesh::FaceIterator fi;
        for(fi = geom->face.begin(); fi!=geom->face.end(); ++fi )
        {
             glBegin(GL_LINE_LOOP);
             glLineWidth(10.0);
             for(int i = 0; i<=3; i++) glVertex((*fi).V(i%3)->P());
             glEnd();
        }
        glEnable(GL_LIGHTING);

    }


    if(selected)
    {
        glDisable(GL_LIGHTING);

        if(subObjectmode)
            glColor3f(0.2, 0.2, 1.0);
        else
            glColor3f(0.2, 1.0, 0.2);

        MyMesh::FaceIterator fi;
        for(fi = geom->face.begin(); fi!=geom->face.end(); ++fi )
        {
             glBegin(GL_LINE_LOOP);
             glLineWidth(10.0);
             for(int i = 0; i<=3; i++) glVertex((*fi).V(i%3)->P());
             glEnd();
        }

        if(subObjectmode)
        {
            float pointSize = 5;

            glColor3f(0.5, 0, 0.5);
            glPointSize(pointSize*0.9);

            glBegin(GL_POINTS);
            // vertices normales
            MyMesh::VertexIterator vi;
            for(vi = geom->vert.begin(); vi!=geom->vert.end(); ++vi )
            {
                 glVertex((*vi).P());
            }
            glEnd();

            glColor3f(1.0, 1.0, 0.0);
            glPointSize(pointSize);
            glBegin(GL_POINTS);
            // vertices seleccionados
            for(unsigned int sel = 0; sel < selectedIds.size(); sel++ )
            {
                glVertex(geom->vert[selectedIds[sel]].P());
            }
            glEnd();

        }
        glEnable(GL_LIGHTING);
    }

	*/

    afterDraw(obj);
}

bool SkeletonRender::updateSkeleton(skeleton* skt)
{
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    skt->root->computeWorldPos();
    glPopMatrix();

    return true;
}


void drawLine(double x, double y, double z)
{
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);

    glVertex3d(0,0,0);
    glVertex3d(x,y,z);

    glEnd();
    glEnable(GL_LIGHTING);
}

void drawBone(double l, double r)
{

    glDisable(GL_LIGHTING);
    glBegin(GL_LINE_LOOP);

    glVertex3d(r,r,r);
    glVertex3d(r,-r,r);
    glVertex3d(r,-r,-r);
    glVertex3d(r,r,-r);
    glVertex3d(r,r,r);  // repetimos el punto inicial para cerrar el círculo.

    glEnd();

    glBegin(GL_LINES);

    glVertex3d(r,r,r);
    glVertex3d(l,0,0);

    glVertex3d(r,-r,r);
    glVertex3d(l,0,0);

    glVertex3d(r,-r,-r);
    glVertex3d(l,0,0);

    glVertex3d(r,r,-r);
    glVertex3d(l,0,0);

    glEnd();
    glEnable(GL_LIGHTING);

}

//JOINT
void JointRender::drawFunc(joint* jt)
{    
    glColor3f(1.0,1.0,1.0);
    glPushMatrix();

    if(jt->father)
    {
        if(jt->father->shading->selected)
            glColor3f((GLfloat)SELR,(GLfloat)SELG,(GLfloat)SELB);
        else
            glColor3f(NORMALR,NORMALG,NORMALB);

        drawLine(jt->pos.X(), jt->pos.Y(),jt->pos.Z());
    }

    glTranslated(jt->pos.X(),jt->pos.Y(),jt->pos.Z());

    glRotatef((GLfloat)jt->orientJoint.Z(),0,0,1);
    glRotatef((GLfloat)jt->orientJoint.Y(),0,1,0);
    glRotatef((GLfloat)jt->orientJoint.X(),1,0,0);

    glRotatef((GLfloat)jt->rot.Z(),0,0,1);
    glRotatef((GLfloat)jt->rot.Y(),0,1,0);
    glRotatef((GLfloat)jt->rot.X(),1,0,0);

    // Pintamos un tri-círculo
    if(selected)
        glColor3f((GLfloat)SELR,(GLfloat)SELG,(GLfloat)SELB);
    else
        glColor3f(NORMALR,NORMALG,NORMALB);

    // Pintamos 3 los círculos
    drawTriCircle(12, DEFAULT_SIZE);

    // Pintamos los ejes del hueso
    drawAxisHandle(DEFAULT_SIZE*0.7);

    // Pintamos la pelota de expansion
    drawExpansionBall(selected, (float)(DEFAULT_SIZE*2*jt->expansion));

    for(unsigned int i = 0; i< jt->childs.size(); i++)
    {
        jt->childs[i]->drawFunc();
    }

    glPopMatrix();
}

void JointRender::computeWorldPos(joint* jt)
{
    glPushMatrix();

    glTranslated(jt->pos.X(),jt->pos.Y(),jt->pos.Z());

    glRotatef((GLfloat)jt->orientJoint.Z(),0,0,1);
    glRotatef((GLfloat)jt->orientJoint.Y(),0,1,0);
    glRotatef((GLfloat)jt->orientJoint.X(),1,0,0);

    glRotatef((GLfloat)jt->rot.Z(),0,0,1);
    glRotatef((GLfloat)jt->rot.Y(),0,1,0);
    glRotatef((GLfloat)jt->rot.X(),1,0,0);

    GLdouble modelview[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
    jt->worldPosition = Point3d(modelview[12],modelview[13],modelview[14]);

    for(unsigned int i = 0; i< jt->childs.size(); i++)
    {
        jt->childs[i]->computeWorldPos();
    }

    glPopMatrix();
}