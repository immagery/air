#include <utils/utilGL.h>
#include "GeometryRender.h"

#include <vcg/complex/allocate.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/update/bounding.h>

#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export.h>
#include "wrap/gl/trimesh.h"

#include <DataStructures/Geometry.h> 

void GeometryRender::drawFunc(object* obj)
{
	
    // transformaciones
    beforeDraw(obj);

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

        //MyMesh::FaceIterator fi;
        //for(fi = geom->face.begin(); fi!= geom->face.end(); ++fi )
		for(int tr = 0; tr< geom->triangles.size(); tr++ )
		{
             //glNormal3dv(&(*fi).N()[0]);
             for(int i = 0; i<3; i++)
             {
				 int pIdx = geom->triangles[tr]->verts[i]->id; //fi->V(i)->IMark();

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
			
                 glNormal(/*fi->V(i)->N()*/ geom->vertNormals[pIdx]);
                 glVertex(geom->nodes[pIdx]->position);
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
			//MyMesh::VertexIterator vi;
			for(int vi = 0; vi < geom->nodes.size(); vi++ )
			//for(vi = geom->vert.begin(); vi!=geom->vert.end(); ++vi )
			{
				int pIdx = geom->nodes[vi]->id;

				//if(colors.size() > 0 && colors[pIdx].size()== 3)
				//	glColor4f(colors[pIdx][0], colors[pIdx][1], colors[pIdx][2], 1.0);
				for(int sv = 0; sv< spotVertexes.size(); sv++)
				{
					if(pIdx == spotVertexes[sv])
					{
						glVertex(geom->nodes[vi]->position);
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
			//MyMesh::VertexIterator vi;
			//for(vi = geom->vert.begin(); vi!=geom->vert.end(); ++vi )
			for(int vi = 0; vi < geom->nodes.size(); vi++ )
			{
				int pIdx = geom->nodes[vi]->id;

				//if(colors.size() > 0 && colors[pIdx].size()== 3)
				//	glColor4f(colors[pIdx][0], colors[pIdx][1], colors[pIdx][2], 1.0);
				if(pIdx == spotVertex)
					glVertex(geom->nodes[vi]->position);
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
        //MyMesh::FaceIterator fi;
		//for(fi = geom->face.begin(); fi!=geom->face.end(); ++fi )
        for(int tr = 0; tr< geom->triangles.size(); tr++ )
		{
             glBegin(GL_LINE_LOOP);
             glLineWidth(10.0);
             for(int i = 0; i<=3; i++) glVertex(geom->triangles[tr]->verts[i%3]->position);
             glEnd();
        }
        glEnable(GL_LIGHTING);

		glDisable(GL_LIGHTING);
		float pointSize = 5;
		glColor3f(0.5, 0, 0.5);
		glPointSize(pointSize*0.9);

		glBegin(GL_POINTS);
		// vertices normales
		//MyMesh::VertexIterator vi;
		//for(vi = geom->vert.begin(); vi!=geom->vert.end(); ++vi )
		for(int vi = 0; vi < geom->nodes.size(); vi++ )
		{
			int pIdx = geom->nodes[vi]->id;
			if(colors.size() > 0 && colors[pIdx].size()== 3)
				glColor4f(colors[pIdx][0], colors[pIdx][1], colors[pIdx][2], 1.0);

			glVertex(geom->nodes[vi]->position);
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

        //MyMesh::FaceIterator fi;
        //for(fi = geom->face.begin(); fi!=geom->face.end(); ++fi )
        for(int tr = 0; tr< geom->triangles.size(); tr++ )
		{
             glBegin(GL_LINE_LOOP);
             glLineWidth(10.0);
             for(int i = 0; i<=3; i++) 
			 {
				glVertex(geom->triangles[tr]->verts[i%3]->position);
			 }
             glEnd();
        }

        if(subObjectmode)
        {
            float pointSize = 5;

            glColor3f(0.5, 0, 0.5);
            glPointSize(pointSize*0.9);

            glBegin(GL_POINTS);
            // vertices normales
            //MyMesh::VertexIterator vi;
            //for(vi = geom->vert.begin(); vi!=geom->vert.end(); ++vi )
            for(int vi = 0; vi < geom->nodes.size(); vi++ )
			{
                glVertex(geom->nodes[vi]->position);
            }
            glEnd();

            glColor3f(1.0, 1.0, 0.0);
            glPointSize(pointSize);
            glBegin(GL_POINTS);
            // vertices seleccionados
            for(unsigned int sel = 0; sel < selectedIds.size(); sel++ )
            {
                glVertex(geom->nodes[selectedIds[sel]]->position);
            }
            glEnd();

        }
        glEnable(GL_LIGHTING);
    }

    afterDraw(obj);
	
}