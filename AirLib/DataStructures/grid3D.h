#ifndef GRID3D_H
#define GRID3D_H

#include "DataStructures.h"
#include "DefNode.h"

#include <vector>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;
using namespace vcg;

class skeleton;

enum T_cell { /*00*/UNTYPED = -1, /*01*/BOUNDARY = -2, /*10*/EXTERIOR = -3, /*11*/INTERIOR = -4 };

class voronoiGraphData
{
public:
    voronoiGraphData()
    {
        voronoiGraph.clear();
        intPoints.clear();
        traductionTable.clear();
    }

    voronoiGraphData(const voronoiGraphData& v)
    {
        intPoints.resize(v.intPoints.size());
        for(unsigned int i = 0; i< v.intPoints.size(); i++)
            intPoints[i] = v.intPoints[i];

        voronoiGraph.resize(v.voronoiGraph.size());
        for(unsigned int i = 0; i< v.voronoiGraph.size(); i++)
        {
            voronoiGraph[i].resize(v.voronoiGraph[i].size());
            for(unsigned int j = 0; j< v.voronoiGraph[i].size(); j++)
            {
                voronoiGraph[i][j] = v.voronoiGraph[i][j];
            }
        }
    }

    vector< vector<bool> > voronoiGraph;

    vector< DefNode > intPoints;
    map< int, DefNode*> nodeIds;

    vector< vector<double> > nodeDistances;
    vector< double > meanDistances;
    vector< vector<double> > embeddedPoints;

    vector< int > traductionTable;

};

// This class represents an influence
class weight
{
public:

	weight()
	{
		label = -1;
		relativeLabel = -1;
		weightValue = 0;
	}

	weight(const weight& temp)
	{
		label = temp.label;
		relativeLabel = temp.relativeLabel;
		weightValue = temp.weightValue;
	}

	weight(int label_in, float weight_in)
	{
		label = label_in;
		relativeLabel = -1;
		weightValue = weight_in;
	}

	weight(int label_in, int relative_label_in, float weight_in)
	{
		label = label_in;
		relativeLabel = relative_label_in;
		weightValue = weight_in;
	}

	int label;
	int relativeLabel; // For secondaryWeights
	float weightValue;
};

int findWeight(vector<weight>& weights, int label);

class cellData
{
    public:
    cellData();
    cellData(int weightsSize);

    void SaveToFile(ofstream& myfile);
    void LoadFromFile(ifstream& myfile);

    void clear();
    void clearAll();

    // Flags for better computation
    Point3i pos;

    // Distance for optimized front propagation
    double distanceToPos;

    // The cell contains a vertex or not
    bool vertexContainer;

    // Weights corresponfin to labels
    //vector<float> weights;

    // Segmentation labels
    //vector<int> labels;

	// Color para representar varias cosas.
	Point3f color;

    // Segmentation labels
    vector<double> embedding;

	// Global owner node of the cell.
    int segmentId;

    // Intermediate owner of the cell, and its weight
    int ownerLabel;
    float ownerWeight;
	float tempOwnerWeight;
	float confidenceLevel;

	// Influences temporary assigned to childs.
	vector<weight> auxInfluences;

    // Domain of influcence
	float domain;
	int domainId;

    // Iteration pass
    int itPass;

	// ConnexComponents
	bool validated;
	int component;
	bool assigned;

	// Influences assigned to this cell
	vector<weight> influences;

    // Aux Weights for optimize computation
    //vector<float> auxWeights;

	float getDomain(int fatherId)
	{
		for(unsigned int i = 0; i< influences.size(); i++)
		{
			if(influences[i].label == fatherId)
				return influences[i].weightValue;
		}

		return 0;
	}
};


class cell3d
{
public:

    //constructors
    cell3d();

    cell3d(bool noData);

    cell3d(int weightsSize);

    cell3d(T_cell tipo, int weightsSize);

    ~cell3d();

    void clear();

    // With these two functions the type is codified as a couple of booleans
    T_cell getType()
    {
        if(tipo[0])
        {
            if(tipo[1]) return INTERIOR;
            else        return EXTERIOR;
        }
        else
        {
            if(tipo[1])  return BOUNDARY;
            else         return UNTYPED;
        }
    }

    void setType(T_cell t)
    {
        if(t == INTERIOR)
        {
            tipo[0] = true; tipo[1] = true;
        }
        else if(t == EXTERIOR)
        {
            tipo[0] = true; tipo[1] = false;
        }
        else if(t == BOUNDARY)
        {
            tipo[0] = false; tipo[1] = true;
        }
        else if(t == UNTYPED)
        {
            tipo[0] = false; tipo[1] = false;
        }
    }


    ///  VARIABLES ///
    // tipo de la celda
    //T_cell tipo;
    bool tipo[2];

    // Has been changed -> inicialized to false (TO REMOVE)
    bool changed;

    bool frontier;

    cellData* data;

    void SaveToFile(ofstream& myfile);
    void LoadFromFile(ifstream& myfile);
};

// Uniform grid
class grid3d
{
public:

    // constructor
    grid3d();

    grid3d(Box3d bounding, Point3i divisions, int weightsSize);

    void initwithNoData(Box3d bounding_, Point3i divisions);
    void init(Box3d bounding, Point3i divisions, int weightsSize);

	void initBasicData();

    void clear();

    void updateStatistics();

    // Dimensiones del grid en los tres ejes (vcg)
    Point3i dimensions;

    // Resolution
    int res;

    // Bounding box del grid (vcg)
    Box3d bounding;

    // Numero de vertices representados en el grid
    int weightSize;

    // tamaño del lado de la celda
    float cellSize;

    // Valores que pueden tomar los pesos del grid.
    float valueRange;

    float kValue;
    float alphaValue;
    bool metricUsed;

	// Vars to track
	float smoothPropagationRatio;
	float worldScale;
	float minConfidenceLevel;

	// binded skeletons
	vector<skeleton*> bindedSkeletons;

	// Node definition data
	voronoiGraphData v;

	// Grid processing temp data
	vector<int> tradIds;

    unsigned int totalCellsCounts;
    unsigned int borderCellsCounts;
    unsigned int inCellsCounts;
    unsigned int outCellsCounts;

    //voronoiGraphData voronoi;

    // Celdas del grid
    vector< vector< vector< cell3d* > > > cells;

    ////  FUNCTIONS ///

    // Marca las celdas con los diferentes tipos según la geometría de entrada.
    int typeCells(MyMesh& mesh);

    // Extiende por interior los pesos asignados en las celdas de los vértices.
    void expandWeights();

    // Normaliza los valores para que est�n entre 0 y 1, como todas las funciones son continuas, esta tambi�n deber�a serlo.
    void normalizeWeights();

	// Normaliza los valores en los vectores auxiliares limitados por el dominio.
	// Solo normaliza las celdas que tienen como propierarios sameLevelIds.
	void normalizeWeightsByDomain();

	// Elimina los pesos igual a cero
	void cleanZeroInfluences();

    // Marca las celdas con los diferentes tipos según la geometría de entrada.
    int typeCells(MyFace& face);

    // Devuelve la direccion de la celda a la que pertenece el punto.
    Point3i cellId(Point3d pt);

	// Devuelve el centro de la celda en coordenadas de mundo.
    Point3d cellCenter(int i,int j,int k);

    // Rellena las celdas que quedan dentro de los contornos.
    // Se llama cuando el contorno ya está inicializado.
    int fillFromCorners();

    // Rellena las celdas que quedan dentro de los contornos.
    // Se llama cuando el resto de celdas ya están inicializadas.
    int fillInside();

    // Guardamos en binario el grid entero, para no tener que recalcular cada vez
    void SaveGridToFile(string sFileName);

    // Cargamos de disco el grid entero.
    void LoadGridFromFile(string sFileName);


	// Report functions
	void ( *fPrintText)(char*);
	void ( *fPrintProcessStatus)(int);

	bool loadedFunctions;

	void SafefPrintText(char* text)
	{
		if(fPrintText) fPrintText(text);
	}
	void SafefPrintProcessStatus(int value)
	{
		if(fPrintProcessStatus) fPrintProcessStatus(value);
	}

};

#endif // GRID3D_H
