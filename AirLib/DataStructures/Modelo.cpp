#include "DataStructures/Modelo.h"
#include "render/GeometryRender.h"
#include "render/ModeloRender.h"
#include "DataStructures/grid3D.h"

//#include "GreenCoords.h"
//#include "HarmonicCoords.h"
//#include "MeanValueCoords.h"

#include <iostream>
#include <fstream>

void Modelo::cleanSpotVertexes()
{
	((GeometryRender*)shading)->spotVertexes.clear();
}

void Modelo::addSpotVertex(int i)
{
	//((GeometryRender*)shading)->spotVertex = i;
	for(int b = 0; b < globalIndirection.size(); b++)
		if(globalIndirection[b]== i)
		{
			((GeometryRender*)shading)->spotVertexes.push_back(globalIndirection[b]);
			return;
		}
}

void Modelo::setSpotVertexes(vector<int>& indices)
{
	((GeometryRender*)shading)->spotVertexes.clear();
	//((GeometryRender*)shading)->spotVertex = i;
	for(int b = 0; b < globalIndirection.size(); b++)
	{
		for(int ind = 0; ind < indices.size(); ind++)
		{
			if(globalIndirection[b]== indices[ind])
			{
				((GeometryRender*)shading)->spotVertexes.push_back(globalIndirection[b]);
			}	
		}
	}
}

void Modelo::setSpotVertex(int i)
{
	//((GeometryRender*)shading)->spotVertex = i;
	for(int b = 0; b < globalIndirection.size(); b++)
		if(globalIndirection[b]== i)
		{
			((GeometryRender*)shading)->spotVertex = globalIndirection[b];
			return;
		}
}

Modelo::Modelo() : Geometry()
{
    // Modelo original que cargamos en memoria.
    sModelPath = "";
    sModelPrefix = "";

    modelCage = NULL;
    dynCage = NULL;

    currentCage = NULL;
    currentRender = this;

    embedding.clear();
	modelVertexDataPoint.clear();
	modelVertexBind.clear();

	bindings.clear();
	computedBindings = false;

	delete shading;
	shading = new ModeloRender(this);
}

Modelo::Modelo(unsigned int nodeId) : Geometry(nodeId)
{
    // Modelo original que cargamos en memoria.
    sModelPath = "";
    sModelPrefix = "";

    modelCage = NULL;
    dynCage = NULL;

    currentCage = NULL;
    currentRender = this;

    embedding.clear();
	modelVertexDataPoint.clear();
	modelVertexBind.clear();

	bindings.clear();
	computedBindings = false;

	delete shading;
	shading = new ModeloRender(this);
}

Modelo::~Modelo()
{
    delete modelCage;
    delete dynCage;

    for(unsigned int i = 0; i< stillCages.size(); i++)
        delete stillCages[i];

    stillCages.clear();

    modelCage = NULL;
    dynCage = NULL;
    currentCage = NULL;
    currentRender = NULL;

	modelVertexDataPoint.clear();
	modelVertexBind.clear();
	globalIndirection.clear();
	embedding.clear();

	for(int i = 0; i< bindings.size(); i++)
		delete bindings[i];
}

void initgrid()
{
	//TODO
}

void Modelo::Clear()
{

    // Modelo original que cargamos en memoria.
    MyMesh::Clear();

    // Cajas para hacer la deformacion
    if(modelCage)
        modelCage->Clear(); // Caja envolvente original

    if(dynCage)
        dynCage->Clear(); // Caja que puede variar interactivamente
    for(unsigned int i = 0; i< stillCages.size(); i++)
    {
        stillCages[i]->Clear();
        delete stillCages[i];
    }

    stillCages.clear();
    sModelPath = "";
    sModelPrefix = "";

    for(unsigned int i = 0; i< embedding.size(); i++)
        embedding[i].clear();

    embedding.clear();

    //PerVertGC.clear();
    //PerFaceGC.clear();
    //PerVertHC.clear();
    //PerVertMVC.clear();

    //HCgrid.clear();

    //newModeloGC.Clear();
    //newModeloHC.Clear();
    //newModeloMVC.Clear();

    currentCage = NULL;
}

void Modelo::drawFunc()
{
    // Pintamos este modelo
	//Geometry::drawFunc();
	((ModeloRender*)shading)->drawFunc(this);

    // Pintamos la caja que toque.
    if(!currentCage)
    {
        if(modelCage)
             currentCage = modelCage;
    }

    shading->beforeDraw(this); // Las cajas y dem·s se mover·n solidarias.

    if(currentCage)
        currentCage->drawFunc();

    shading->afterDraw(this);
}

void Modelo::select(bool bToogle, unsigned int id)
{
    //bool lo_tengo = false;
	bToogle = bToogle;

    if(shading) shading->selected = ( id == nodeId);

    if(modelCage && modelCage->shading != NULL)
    {
        bool sel = id == modelCage->nodeId;
        modelCage->shading->selected = sel;
        modelCage->shading->visible = sel;

        if(sel)
            currentCage = modelCage;
    }
    if(dynCage && dynCage->shading != NULL)
    {
        bool sel = id == dynCage->nodeId;
        dynCage->shading->selected = sel;
        dynCage->shading->visible  = sel;

        if(sel)
            currentCage = dynCage;
    }

    for(unsigned int i = 0; i< stillCages.size(); i++)
    {
        if(stillCages[i]->shading)
        {
            bool sel = id == stillCages[i]->nodeId;
            stillCages[i]->shading->selected = sel;
            ((Cage*)stillCages[i])->shading->visible = sel;

            if(sel)
                currentCage = stillCages[i];
        }
    }
}

/*
bool Modelo::processGreenCoordinates()
{

    newModeloGC.Clear();
    vcg::tri::Append<MyMesh, MyMesh>::Mesh(newModeloGC,modeloOriginal, false);

    string sGCSavedCoords = sModelPath;
    sGCSavedCoords.append("/").append(name).append("_").append(GC_COMP_FILE_NAME);

    ifstream myfile;
    myfile.open (sGCSavedCoords.c_str(), ios::in |ios::binary);
    if (myfile.is_open())
    {
        myfile.close();
        LoadGCCoordsFromFile(PerVertGC, PerFaceGC, sGCSavedCoords);
        return false;
    }
    else
    {
        myfile.close();
        gpCalculateGreenCoordinates( modeloOriginal, cage, PerVertGC, PerFaceGC, sGCSavedCoords);
    }

    return true;

}
*/
/*
bool Modelo::processHarmonicCoordinates()
{
    newModeloHC.Clear();
    vcg::tri::Append<MyMesh, MyMesh>::Mesh(newModeloHC,modeloOriginal, false);

    //unsigned int resolution = pow(2,7);
    unsigned int resolution = pow(2,7);
    string sHCSavedGrid = sModelPath;
    sHCSavedGrid.append("/").append(name).append("_").append(HC_GRID_FILE_NAME);

    ifstream myfile;
    myfile.open (sHCSavedGrid.c_str(), ios::in |ios::binary);
    if (myfile.is_open())
    {
        myfile.close();
        loadHarmonicCoordinates(modeloOriginal, HCgrid, PerVertHC, sHCSavedGrid);
                return false;
    }
    else
    {
        myfile.close();
        getHarmonicCoordinates(modeloOriginal, cage, HCgrid, PerVertHC, resolution, sHCSavedGrid);
    }

    return true;
}
*/

/*
bool Modelo::processMeanValueCoordinates()
{
    newModeloMVC.Clear();
    vcg::tri::Append<MyMesh, MyMesh>::Mesh(newModeloMVC,modeloOriginal, false);

    string sMVCSavedGrid = sModelPath;
    sMVCSavedGrid.append("/").append(name).append("_").append(MVC_COMP_FILE_NAME);
    ifstream myfile;
    myfile.open (sMVCSavedGrid.c_str(), ios::in |ios::binary);
    if (myfile.is_open())
    {
        myfile.close();
        LoadMVCoordsFromFile(PerVertMVC, sMVCSavedGrid);
                return false;
    }
    else
    {
        myfile.close();
        CalculateMeanValueCoords( modeloOriginal, cage, PerVertMVC, sMVCSavedGrid);
    }

    return true;
}
*/

/*
bool Modelo::deformMesh(int cageId, coordsType coords)
{
    // Ojo!-> si no se han computado las coordenadas antes -> ¡¡ERROR!!

    // Primero escogemos la caja con la que queremos hacer la deformación
    MyMesh* newCage;
    if(cageId < 0 || cageId >= (int)stillCages.size()) // Caso normal
        newCage = &dynCage;
    else
    {
        newCage = stillCages[cageId];
    }

    if(coords == HARMONIC_COORDS)
    {
        deformMeshWithHC(modeloOriginal, cage, newModeloHC, *newCage, PerVertHC);
    }
    else if(coords == GREEN_COORDS)
    {
        deformMeshWithGC(modeloOriginal, cage, newModeloGC, *newCage, PerVertGC, PerFaceGC);
    }
    else if(coords == MEANVALUE_COORDS)
    {
        deformMeshWithMVC(modeloOriginal, cage, newModeloMVC, *newCage, PerVertMVC);
    }
    else if(coords == ALL_COORDS)
    {
        // Green Coordinates
        deformMeshWithGC(modeloOriginal, cage, newModeloGC, *newCage, PerVertGC, PerFaceGC);

        // Harmonic Coordinates
        deformMeshWithHC(modeloOriginal, cage, newModeloHC, *newCage, PerVertHC);

        //Mean Value Coordinates
        deformMeshWithMVC(modeloOriginal, cage, newModeloMVC, *newCage, PerVertMVC);
    }

    return true;
}
*/
