#ifndef GEOMETRY_H
#define GEOMETRY_H

#define DEBUG false

#include "Object.h"
#include "DataStructures.h"

using namespace std;

// GEOMETRY
class Geometry : public object , public MyMesh
{    
    public:
        Geometry();
		Geometry(unsigned int nodeId);
		~Geometry();

        void loadModel(string fileName, string name, string ext, string path);
        void saveModel(string fileName, string name, string ext, string path);
        bool cleanModel();

        virtual void drawFunc();

        virtual void freezeTransformations();

        virtual Point3d getSelCenter();
        virtual bool getBoundingBox(Point3d& minAuxPt,Point3d& maxAuxPt);

};

#endif // GEOMETRY_H
