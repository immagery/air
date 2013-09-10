#include "clipingPlaneRender.h"
#include <utils/utilGL.h>

void drawQuad4Colors(vector<Point3d>& points,vector<Point3f>&  colors, int idx01, int idx02, int idx03, int idx04)
{     
    glDisable(GL_CULL_FACE);
    glDisable(GL_LIGHTING);
    glBegin(GL_QUADS);
        
	glColor3fv(&colors[idx01][0]);
    glVertex3dv(&points[idx01][0]);
	   
	glColor3fv(&colors[idx02][0]);
    glVertex3dv(&points[idx02][0]);

	glColor3fv(&colors[idx03][0]);
    glVertex3dv(&points[idx03][0]);

	glColor3fv(&colors[idx04][0]);
    glVertex3dv(&points[idx04][0]);

    glEnd();

    glEnable(GL_CULL_FACE);
    glEnable(GL_LIGHTING);
}

void clipingPlaneRenderer::drawFunc(object* obj)
{
    if(!dirtyFlag)
    {
        //updateSlices();
        dirtyFlag = false;
    }

	beforeDraw(plane);

	assert(subdivisions*subdivisions == points.size());
	if(posUpdated)
	{
		for(int i = 0; i< subdivisions-1; i++)
		{
			for(int j = 0; j< subdivisions-1; j++)
			{
				int idx = i*subdivisions+j;
				drawQuad4Colors(points, colors, idx, idx+1, idx+subdivisions+1, idx+subdivisions);
			}
		}
	}
	else
	{
		if(points.size() == 0)
			return;

		vector<Point3f> colorMate;
		colorMate.resize(points.size());
		for(int pt = 0; pt < points.size(); pt++)
			colorMate[pt] = Point3f(0.2,0.8,0.2);
		for(int i = 0; i< subdivisions-1; i++)
		{
			for(int j = 0; j< subdivisions-1; j++)
			{
				int idx = i*subdivisions+j;
				drawQuad4Colors(points, colorMate, idx, idx+1, idx+subdivisions+1, idx+subdivisions);
			}
		}
	}

	afterDraw(plane);
}


void clipingPlaneRenderer::beforeDraw(object* obj)
{
	shadingNode::beforeDraw(obj);
		//if(dirtyFlag) 
		//	update(obj);
}

bool clipingPlaneRenderer::update(object* obj)
{
    if(!dirtyFlag)
        return true;
    else
    {
        //updateSlices();
        dirtyFlag = false;
    }

    return true;
}
