//#ifdef WIN32
//#include <GL/glew.h>
//#include <GL/gl.h>
//#include <GL/glu.h>
//#endif

//#ifdef MACOSX
//#include <gl.h>
//#include <glu.h>
//#endif

#include "Geometry.h"
#include "..\render\geometryRender.h"

#include <vcg/complex/allocate.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/update/bounding.h>

#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export.h>
//#include "wrap/gl/trimesh.h"

Geometry::Geometry() : object()
{
	shading = new GeometryRender(this);
}

Geometry::Geometry(unsigned int nodeId) : object(nodeId)
{
	shading = new GeometryRender(this);
}

Geometry::~Geometry()
{
	delete shading;
}

void Geometry::drawFunc()
{
    // transformaciones
	if(!shading)
		return;
		
	//shading->beforeDraw(this);
	((GeometryRender*)shading)->drawFunc(this);
    //shading->afterDraw(this);
}

void Geometry::freezeTransformations()
{
    Matrix44d tm(tMatrix);

	for(int vi = 0; vi < nodes.size(); vi++ )
		nodes[vi]->position = tm*nodes[vi]->position;

    loadIdentity();
}

bool Geometry::cleanModel()
{
	nodes.clear();
	triangles.clear();

	return true;
}

// Guarda un modelo de disco
// filename: fichero de ruta completa.
void Geometry::saveModel(string fileName, string name, string ext, string path)
{

    if(ext.compare("off") == 0)
    {
        //vcg::tri::io::ExporterOFF<MyMesh>::Save(*this,fileName.c_str());
    }
    else if(ext.compare("ply") == 0)
    {
        printf("PlyFile: to do SaveModel\n"); fflush(0);
           //model.Clear();
           //vcg::tri::io::ImporterPLY<MyMesh>::Open(model,fileName.toStdString().c_str());
           //gpUpdateNormals(model, false);
           //loadedModel = true;
    }
    else if(ext.compare("3ds") == 0)
    {
        printf("3DsFile: to do SaveModel\n"); fflush(0);
        /*
        m_.modeloOriginal.Clear();
        m_.dynCage.Clear();
        m_.cage.Clear();
        //vcg::tri::io::ImporterOBJ<MyMesh>::Open(model,fileName.toStdString().c_str());
        gpUpdateNormals(m_.modeloOriginal, false);
        loadedModel = true;
        */
    }
    else
    {
        printf("Aviso: La extension del ficehero no es conocida\n"); fflush(0);
        return;
    }

}

// Lee un modelo de disco
// filename: fichero de ruta completa.
// model: MyMesh donde se almacenara el modelo leido.
void Geometry::loadModel(string fileName, string name, string ext, string path)
{
    this->sPath = path;
    this->sName = name;

    dirtyFlag = true;

	if(ext.compare("obj") == 0)
    {
		/*
		vcg::tri::io::ImporterOBJ<MyMesh>::Info infoImp;
		vcg::tri::io::ImporterOBJ<MyMesh>::Open(*this,fileName.c_str(), infoImp);

		vcg::tri::UpdateTopology<MyMesh>::VertexFace(*this);
        vcg::tri::UpdateBounding<MyMesh>::Box(*this);

        gpUpdateNormals(*this, true);
        dirtyFlag = false;
		*/
	}
    else if(ext.compare("off") == 0)
    {
		BuildSurfaceFromOFFFile(*this, fileName);
		computeNormals();
        dirtyFlag = false;
    }
    else if(ext.compare("ply") == 0)
    {
           //model.Clear();
           //vcg::tri::io::ImporterPLY<MyMesh>::Open(model,fileName.toStdString().c_str());
           //gpUpdateNormals(model, false);
           //loadedModel = true;
    }
    else if(ext.compare("3ds") == 0)
    {
        printf("3DsFile: to do LoadModel\n"); fflush(0);
        /*
        m_.modeloOriginal.Clear();
        m_.dynCage.Clear();
        m_.cage.Clear();
        //vcg::tri::io::ImporterOBJ<MyMesh>::Open(model,fileName.toStdString().c_str());
        gpUpdateNormals(m_.modeloOriginal, false);
        loadedModel = true;
        */
    }
    else
    {
        printf("Aviso: La extension del ficehero no es conocida\n"); fflush(0);
        return;
    }

}


// Return a bool if there is a bounding box computable
bool Geometry::getBoundingBox(Point3d& minAuxPt,Point3d& maxAuxPt)
{
	if(nodes.size() < 0)
	{
		minBBox = maxBBox = minAuxPt = maxAuxPt = Point3d(0,0,0);
		return false;
	}

	double minValue[3], maxValue[3];	

	for(int i = 0; i< 3; i++)
		minValue[i] = maxValue[i] = nodes[0]->position[i];

	for(int i = 1; i< nodes.size(); i++)
	{
		for(int j = 0; j< 3; j++)
		{
			minValue[j] = min(minValue[j], nodes[i]->position[j]);
			maxValue[j] = max(maxValue[j], nodes[i]->position[j]);
		}
	}

    minBBox = minAuxPt = Point3d(minValue[0], minValue[1], minValue[2]);
    maxBBox = maxAuxPt = Point3d(maxValue[0], maxValue[1], maxValue[2]);
    return true;
}

void Geometry::computeFaceNormals()
{
	faceNormals.resize(triangles.size());
	for(int i = 0; i < triangles.size(); i++)
	{
		Point3d v1 = triangles[i]->verts[1]->position - triangles[i]->verts[0]->position;
		Point3d v2 = triangles[i]->verts[2]->position - triangles[i]->verts[0]->position;
		faceNormals[i] = v1.normalized()^v2.normalized();		
	}
}

void Geometry::computeVertNormals()
{
	if(triangles.size() != faceNormals.size())
		computeFaceNormals();

	vertNormals.resize(nodes.size());
	vector<int> vertTriCounter;
	vertTriCounter.resize(nodes.size(),0);

	for(int i = 0; i < nodes.size();i++)
		vertNormals[i] = Point3d(0,0,0);

	for(int i = 0; i < triangles.size(); i++)
	{
		for(int j = 0; j < triangles[i]->verts.size(); j++)
		{
			vertTriCounter[triangles[i]->verts[j]->id]++;
			vertNormals[triangles[i]->verts[j]->id] += faceNormals[i];
		}
	}

	for(int i = 0; i < nodes.size();i++)
		vertNormals[i] = (vertNormals[i]/vertTriCounter[i]).normalized();

	//faceNormals.clear();
}

void Geometry::computeNormals()
{
	computeFaceNormals();
	computeVertNormals();
}

Point3d Geometry::getSelCenter()
{
    if(shading->subObjectmode)
    {
        Point3d center;
        for(unsigned int i = 0; i>shading->selectedIds.size(); i++)
        {
			assert(false);
            //if(i == 0)
            //    center = vert[shading->selectedIds[i]].P();
            //else
            //    center += vert[shading->selectedIds[i]].P();
        }

        if(shading->selectedIds.size() > 0)
            center = center/shading->selectedIds.size();

        return center;
    }

    return pos;
}
