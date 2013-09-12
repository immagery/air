#ifndef GEOMETRY_H
#define GEOMETRY_H

#define DEBUG false

#include "Object.h"
#include "DataStructures.h"

#include "SurfaceData.h"

using namespace std;

// GEOMETRY
class Geometry : public object , public SurfaceGraph
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

		Point3d minBBox;
		Point3d maxBBox;

		void computeFaceNormals();
		void computeVertNormals();
		void computeNormals();

		vector<Point3d> faceNormals;
		vector<Point3d> vertNormals;

};

#endif // GEOMETRY_H
