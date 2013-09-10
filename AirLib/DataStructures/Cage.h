#ifndef CAGE_H
#define CAGE_H

#include "Geometry.h"

enum cageType { DYN_CAGE = 0, STILL_CAGE, MODEL_CAGE};

class Cage : public Geometry
{
public:
    cageType type;

    Cage() : Geometry()
    {
      type = MODEL_CAGE;
      setDefaultShading();
    }

    Cage(unsigned int nodeId) : Geometry(nodeId)
    {
      type = MODEL_CAGE;
      setDefaultShading();
    }

    void setDefaultShading()
    {
        // Shading standard for cages
        shading->shtype = T_LINES;
        shading->visible = false;

        shading->color[0] = (float)1.0;
        shading->color[1] = (float)0.2;
        shading->color[2] = (float)0.2;
    }
};


#endif // CAGE_H
