#ifndef GEOMETRY_RENDER_H
#define GEOMETRY_RENDER_H

#define DEBUG false

#include "ShadingNode.h"

using namespace std;

// GEOMETRY
class Geometry;
class object;
class GeometryRender : public shadingNode
{    
    public:
		GeometryRender() : shadingNode()
        {
			geom = NULL;
			spotVertex = 0;
        }

        GeometryRender(Geometry* g) : shadingNode()
        {
			geom = g;
			spotVertex = 0;
        }

		Geometry* geom;
		int spotVertex;
		vector<int> spotVertexes;

        virtual void drawFunc(object* obj);
};

#endif // GEOMETRY_RENDER_H
