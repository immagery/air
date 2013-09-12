#include "Deformer.h"
#include "Modelo.h"

// DEFORMER DIRTY PROPAGATION
bool deformer::propagateDirtyness()
{
    dirtyFlag = true;

    if(targetModel)
        targetModel->propagateDirtyness();

    return true;
}

// DEFORMER DIRTY PROPAGATION
bool CageDeformer::update()
{
    if(sourceModel->dirtyFlag)
        sourceModel->update();

    processCoords();
    deformMesh();

    dirtyFlag = false;

    return true;
}



// Mean Value Coordinates
MVCDeformer::MVCDeformer()
{
    CageDeformer(MEANVALUE_COORDS);
}

MVCDeformer::MVCDeformer(Modelo* inGeom1, Modelo* inGeom2)
{
    CageDeformer(MEANVALUE_COORDS, inGeom1, inGeom2);
}

bool MVCDeformer::processCoords()
{
	/*
    targetModel->Clear();
    vcg::tri::Append<MyMesh, MyMesh>::Mesh(*targetModel,*sourceModel, false);

    string sMVCSavedGrid = sourceModel->sModelPath;
    sMVCSavedGrid.append("/").append(sourceModel->sName).append("_").append(MVC_COMP_FILE_NAME);
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
        CalculateMeanValueCoords( *sourceModel, *(sourceModel->modelCage), PerVertMVC, sMVCSavedGrid);
    }
	*/

    return true;
}

bool MVCDeformer::deformMesh()
{
	/*
    if(dirtyFlag)
    {
        // hay que computar las coordenadas, propagar la actualización hacia arriba
    }

    MyMesh* newCage = targetModel->modelCage; //para un modelo target, su cage es un puntero al objetivo.
    if(newCage == NULL) return false;

    deformMeshWithMVC(*sourceModel, *sourceModel->modelCage, *targetModel, *newCage, PerVertMVC);

	*/

    // Este nodo está actualizado
    dirtyFlag = true;

    return true;
}

// Harmonic Coordinates
HCDeformer::HCDeformer()
{
    CageDeformer(HARMONIC_COORDS);
}

HCDeformer::HCDeformer(Modelo* inGeom1, Modelo* inGeom2)
{
    CageDeformer(HARMONIC_COORDS, inGeom1, inGeom2);
}

bool HCDeformer::processCoords()
{
	assert(false);
	/*
    if(!sourceModel || !targetModel)
        return false;

    targetModel->Clear();
    vcg::tri::Append<MyMesh, MyMesh>::Mesh(*targetModel,*sourceModel, false);

    //unsigned int resolution = pow(2,7);
    unsigned int resolution = (int)pow(2.0,7);
    string sHCSavedGrid = sourceModel->sModelPath;
    sHCSavedGrid.append("/").append(sourceModel->sName).append("_").append(HC_GRID_FILE_NAME);

    ifstream myfile;
    myfile.open (sHCSavedGrid.c_str(), ios::in |ios::binary);
    if (myfile.is_open())
    {
        myfile.close();
        //deformMeshWithHC(*sourceModel, cage, newModeloHC, *newCage, PerVertHC);
        //loadHarmonicCoordinates(*sourceModel, HCgrid, PerVertHC, sHCSavedGrid);
        return false;
    }
    else
    {
        myfile.close();
        //getHarmonicCoordinates(*sourceModel, *(sourceModel->modelCage), HCgrid, PerVertHC, resolution, sHCSavedGrid);
    }

	*/
    return true;
}

bool HCDeformer::deformMesh()
{
    if(dirtyFlag)
    {
        // hay que computar las coordenadas, propagar la actualización hacia arriba
    }

	assert(false);
	/*
    MyMesh* newCage = targetModel->modelCage; //para un modelo target, su cage es un puntero al objetivo.
    if(newCage == NULL) return false;

    deformMeshWithHC(*sourceModel, *(sourceModel->modelCage), *targetModel, *newCage, PerVertHC);
	*/

    // Este nodo está actualizado
    dirtyFlag = true;
	

    return true;
}

// Green Coordinates
GCDeformer::GCDeformer()
{
    CageDeformer(GREEN_COORDS);
}

GCDeformer::GCDeformer(Modelo* inGeom1, Modelo* inGeom2)
{
    CageDeformer(GREEN_COORDS, inGeom1, inGeom2);
}

bool GCDeformer::processCoords()
{
	assert(false);
	/*
    if(!sourceModel || !targetModel)
        return false;

    targetModel->Clear();
    vcg::tri::Append<MyMesh, MyMesh>::Mesh(*targetModel,*sourceModel, false); // Quizás no funciona porque no es exactamente MyMesh

    string sGCSavedCoords = sourceModel->sModelPath;
    sGCSavedCoords.append("/").append(sourceModel->sName).append("_").append(GC_COMP_FILE_NAME);

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
        gpCalculateGreenCoordinates( *sourceModel, *(sourceModel->modelCage), PerVertGC, PerFaceGC, sGCSavedCoords);
    }

	*/
    return true;
}

bool GCDeformer::deformMesh()
{
	assert(false);
	/*
    if(dirtyFlag)
    {
        // hay que computar las coordenadas, propagar la actualización hacia arriba
    }

    MyMesh* newCage = targetModel->modelCage; //para un modelo target, su cage es un puntero al objetivo.
    if(newCage == NULL) return false;

    deformMeshWithGC(*sourceModel, *(sourceModel->modelCage), *targetModel, *newCage, PerVertGC, PerFaceGC);

    // Este nodo está actualizado
    dirtyFlag = true;

	*/
    return true;
}
