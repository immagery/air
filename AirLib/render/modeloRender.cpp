#include <utils/utilGL.h>
#include "ModeloRender.h"
#include "ShadingNode.h"

#include <vcg/complex/allocate.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/update/bounding.h>

#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export.h>
#include "wrap/gl/trimesh.h"

#include <DataStructures/Modelo.h> 

void ModeloRender::drawFunc(object* obj)
{
	GeometryRender::drawFunc( obj);

	// transformaciones
	beforeDraw(obj);

	// Pintamos detalles de los que queramos marcar.
    glBegin(GL_TRIANGLES);
	
	Modelo* m = (Modelo*)obj;

	glDisable(GL_LIGHTING);
	glColor3f(1.8, 0.2, 0.3);
	for(unsigned int bds = 0; bds < m->bindings.size(); bds++ )
    {
		for(unsigned int trs = 0; trs < m->bindings[bds]->virtualTriangles.size(); trs++ )
		{
			glVertex( m->bindings[bds]->virtualTriangles[trs].pts[0]->position);
			glVertex( m->bindings[bds]->virtualTriangles[trs].pts[1]->position);
			glVertex( m->bindings[bds]->virtualTriangles[trs].pts[2]->position);
		}
	}
    glEnable(GL_LIGHTING);

	glEnd();

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
					 {
						float r = colors[pIdx][0]*blend;
						float g = colors[pIdx][1]*blend;
						float b = colors[pIdx][2]*blend;

                        glColor4f(colors[pIdx][0]*blend, colors[pIdx][1]*blend, colors[pIdx][2]*blend,blend);
					 }
                     else
                        glColor4f(color[0]*blend, color[1]*blend, color[2]*blend,blend);
                 }

                 glNormal(fi->V(i)->N());
                 glVertex(fi->V(i)->P());
             }
         }
        glEnd();

		//if(m_bDrawPoints)
		//{

		if(spotVertexes.size() > 0)
		{
			glDisable(GL_LIGHTING);
			float pointSize = 8;
			glColor3f(0.9, 0.1, 0.1);
			glPointSize(pointSize*0.9);

			glBegin(GL_POINTS);
			int countSV= 0;
			// vertices normales
			MyMesh::VertexIterator vi;
			for(vi = geom->vert.begin(); vi!=geom->vert.end(); ++vi )
			{
				int pIdx = vi->IMark();

				//if(colors.size() > 0 && colors[pIdx].size()== 3)
				//	glColor4f(colors[pIdx][0], colors[pIdx][1], colors[pIdx][2], 1.0);
				for(int sv = 0; sv< spotVertexes.size(); sv++)
				{
					if(pIdx == spotVertexes[sv])
					{
						glVertex((*vi).P());
						countSV++;
						break;
					}
				}
				if(countSV == spotVertexes.size()) break;
			}
			glEnd();
			glEnable(GL_LIGHTING);
		}
		//}

		if(spotVertex >= 0)
		{
			glDisable(GL_LIGHTING);
			float pointSize = 8;
			glColor3f(0.1, 0.9, 0.1);
			glPointSize(pointSize*0.9);

			glBegin(GL_POINTS);
			// vertices normales
			MyMesh::VertexIterator vi;
			for(vi = geom->vert.begin(); vi!=geom->vert.end(); ++vi )
			{
				int pIdx = vi->IMark();

				//if(colors.size() > 0 && colors[pIdx].size()== 3)
				//	glColor4f(colors[pIdx][0], colors[pIdx][1], colors[pIdx][2], 1.0);
				if(pIdx == spotVertex)
					glVertex((*vi).P());
			}
			glEnd();
			glEnable(GL_LIGHTING);
		}

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

		glDisable(GL_LIGHTING);
		float pointSize = 5;
		glColor3f(0.5, 0, 0.5);
		glPointSize(pointSize*0.9);

		glBegin(GL_POINTS);
		// vertices normales
		MyMesh::VertexIterator vi;
		for(vi = geom->vert.begin(); vi!=geom->vert.end(); ++vi )
		{
			int pIdx = vi->IMark();
			if(colors.size() > 0 && colors[pIdx].size()== 3)
				glColor4f(colors[pIdx][0], colors[pIdx][1], colors[pIdx][2], 1.0);
			glVertex((*vi).P());
		}
		glEnd();
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