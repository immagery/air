#ifndef DEFORMER_H
#define DEFORMER_H

#include "cage.h"
#include "grid3D.h"
#include "Node.h"

#include "..\Computation\GreenCoords.h"
#include "..\Computation\HarmonicCoords.h"
#include "..\Computation\MeanValueCoords.h"

#define DYN_CAGE_ID -1

#define MVC_COMP_FILE_NAME "mvc_computation_copy.bin"
#define GC_COMP_FILE_NAME "gc_computation_copy.bin"
#define HC_GRID_FILE_NAME "hc_grid_copy.bin"

enum coordsType { GREEN_COORDS = 0, HARMONIC_COORDS, MEANVALUE_COORDS, STAR_CAGES, GENERAL, ALL_COORDS};

class Modelo;

class deformer : public node
{
    public:
    // Updated Node Flag
    bool dirtyFlag;

    // plugs de entrada
    Modelo* sourceModel;

    // plugs de salida
    Modelo* targetModel; // Result of deformation

    // Constructors
    deformer()
    {
        dirtyFlag = true;
        sourceModel = NULL;
        targetModel = NULL;
    }

    deformer(Modelo* inGeom1, Modelo* inGeom2)
    {
        dirtyFlag = true;

        if(!inGeom1)
        {
            printf("El modelo de entrada no existe. (Deformer)"); fflush(0);
            assert(false);
            return;
        }

        if(!inGeom2)
        {
            printf("El modelo de salida no existe. (Deformer)"); fflush(0);
            assert(false);
            return;
        }

        sourceModel = inGeom1;
        targetModel = inGeom2;
    }

    // Set Plugs
    void setSourceModel(Modelo* inGeom)
    {
        if(!inGeom)
        {
            printf("El modelo de entrada no existe. (Deformer)"); fflush(0);
            assert(false);
            return;
        }

        sourceModel = inGeom;
    }
    void setTargetModel(Modelo* inGeom)
    {
        if(!inGeom)
        {
            printf("El modelo de salida no existe. (Deformer)"); fflush(0);
            assert(false);
            return;
        }

        targetModel = inGeom;
    }

    // Processing functions
    bool process() {return false;}
    bool deformMesh(){ return false;}

    virtual bool propagateDirtyness();
    virtual bool update(){ return false; }

};


class CageDeformer : public deformer
{
    public:

    coordsType type;
    string coordsName;

    CageDeformer()
    {
      type = MEANVALUE_COORDS;
    }

    CageDeformer(coordsType _type)
    {
        type = _type;
        deformer();
    }

    CageDeformer(coordsType _type, Modelo* inGeom1, Modelo* inGeom2)
    {
        type = _type;
        deformer( inGeom1, inGeom2);
    }

    string getCoordsName()
    {
        switch(type)
        {
            case GREEN_COORDS:
                return "green";
            case HARMONIC_COORDS:
                return "harmonic";
            case MEANVALUE_COORDS:
                return "meanValue";
            case STAR_CAGES:
                return "starCage";
            default:
                return "general";
        }
    }
    virtual bool update();
    virtual bool processCoords(){return true;}
    virtual bool deformMesh(){return true;}

};

class HCDeformer : public CageDeformer
{
    public:
        // Harmonic Coordinates
        grid3d HCgrid;
        vector< vector<float> > PerVertHC;

        HCDeformer();
        HCDeformer(Modelo* inGeom1, Modelo* inGeom2);

        virtual bool processCoords();
        virtual bool deformMesh();

};

class MVCDeformer : public CageDeformer
{
    public:

    // Mean Value Coordinates
    vector< vector<float> > PerVertMVC;

    MVCDeformer();
    MVCDeformer(Modelo* inGeom1, Modelo* inGeom2);

    virtual bool processCoords();
    virtual bool deformMesh();

};

class GCDeformer : public CageDeformer
{
    public:

        // Green Coordinates
        vector< vector<float> > PerVertGC;
        vector< vector<float> > PerFaceGC;

        GCDeformer();
        GCDeformer(Modelo* inGeom1, Modelo* inGeom2);

        virtual bool processCoords();
        virtual bool deformMesh();
};

// bool processGreenCoordinates();
// bool processHarmonicCoordinates();
// bool processMeanValueCoordinates();
// bool deformMesh(int cageId, coordsType coords = ALL_COORDS);

#endif // DEFORMER_H
