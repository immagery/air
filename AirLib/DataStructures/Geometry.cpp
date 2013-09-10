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

Geometry::Geometry() : object(), MyMesh()
{
	shading = new GeometryRender(this);
}

Geometry::Geometry(unsigned int nodeId) : object(nodeId), MyMesh()
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
    MyMesh::VertexIterator vi;
    for(vi = vert.begin(); vi!=vert.end(); ++vi ) {
        Point3d pt = vi->P();
        pt = tm*pt;
        vi->P() = pt;
     }

    loadIdentity();
}

bool Geometry::cleanModel()
{
    vector<bool> used(vn, false);

    MyMesh::FaceIterator fi;
    for(fi = face.begin(); fi!=face.end(); ++fi )
    {
        for(int vi = 0; vi < 3; vi++)
        {
            used[(*fi).V(vi)->IMark()] = true;
        }
    }

    bool cleaned = false;
    MyMesh::VertexIterator vi;
    for(vi = vert.begin(); vi!=vert.end(); ++vi )
    {
        if(!used[(*vi).IMark()])
        {
            //vi->setD();
            cleaned = true;
        }
    }

    return cleaned;
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
		vcg::tri::io::ImporterOBJ<MyMesh>::Info infoImp;
		vcg::tri::io::ImporterOBJ<MyMesh>::Open(*this,fileName.c_str(), infoImp);

		vcg::tri::UpdateTopology<MyMesh>::VertexFace(*this);
        vcg::tri::UpdateBounding<MyMesh>::Box(*this);

        gpUpdateNormals(*this, true);
        dirtyFlag = false;
	}
    else if(ext.compare("off") == 0)
    {
        vcg::tri::io::ImporterOFF<MyMesh>::Open(*this,fileName.c_str());
		vcg::tri::UpdateTopology<MyMesh>::VertexFace(*this);
		vcg::tri::UpdateBounding<MyMesh>::Box(*this);

        gpUpdateNormals(*this, true);
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
    minAuxPt = bbox.min;
    maxAuxPt = bbox.max;
    return true;
}

Point3d Geometry::getSelCenter()
{
    if(shading->subObjectmode)
    {
        Point3d center;
        for(unsigned int i = 0; i>shading->selectedIds.size(); i++)
        {
            if(i == 0)
                center = vert[shading->selectedIds[i]].P();
            else
                center += vert[shading->selectedIds[i]].P();
        }

        if(shading->selectedIds.size() > 0)
            center = center/shading->selectedIds.size();

        return center;
    }

    return pos;
}
