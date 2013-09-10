#ifndef MODELO_RENDER_H
#define MODELO_RENDER_H

#define DEBUG false

#include "GeometryRender.h"
using namespace std;

class GeometryRender;
class object;
class ModeloRender : public GeometryRender
{    
    public:
		ModeloRender() : GeometryRender()
        {
        }

        ModeloRender(Geometry* g) : GeometryRender(g)
        {
        }

        virtual void drawFunc(object* obj);
};

#endif //MODELO_RENDER_H