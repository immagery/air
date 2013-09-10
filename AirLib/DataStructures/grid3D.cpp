#include "..\DataStructures\grid3D.h"
#include "..\DataStructures\DefNode.h"
#include "..\utils\util.h"

#define DEBUG false

#include <iostream>
#include <fstream>

using namespace std;
using namespace vcg;

int findWeight(vector<weight>& weights, int label)
{
	for(int i = 0; i< weights.size(); i++)
	{
		if(weights[i].label == label)
			return i;
	}

	return -1;
}

//////////////////////////////////////
///////////// CELL DATA //////////////

cellData::cellData()
{
    influences.clear();
    auxInfluences.clear();

    embedding.clear();
    vertexContainer = false;
    pos = Point3i(-1,-1,-1);
    itPass =-1;

	confidenceLevel = 0;
    
	ownerLabel = -1;
	ownerWeight = 0;
	confidenceLevel = 0;

	distanceToPos = -1;

	segmentId = -1;

	domain = 0.0;
	domainId = -1;

	component = -1;
	assigned = false;
	validated = false;

	color = Point3f(0,0,0);
}

cellData::cellData(int weightsSize)
{
    if(weightsSize > 0)
        influences.resize(weightsSize);
    else
        influences.clear();

    vertexContainer = false;

    auxInfluences.clear();
    pos = Point3i(-1,-1,-1);
    itPass =-1;
    ownerLabel = -1;
	confidenceLevel = 0;

	component = -1;
	assigned = false;
	validated = false;

	color = Point3f(0,0,0);
}

// Clear no del todo, no quitamos el labeling de la voxelizacion
void cellData::clear()
{
    auxInfluences.clear();
    influences.clear();

	component = -1;
	assigned = false;

	confidenceLevel = 0;

    pos = Point3i(-1,-1,-1);
    itPass =-1;

    // Flags for better computation
    distanceToPos = -1;

	// TO DEBUG
	//vertexContainer;
	//Point3f color;
	color = Point3f(0,0,0);

	// No tocaria
    //vector<double> embedding;
	
	ownerLabel = -1;
	ownerWeight = 0;
	confidenceLevel = 0;

	segmentId = -1;

	domain = 0.0;
	domainId = -1;

	component = -1;
	assigned = false;
	validated = false;
}

void cellData::clearAll()
{
    clear();
    embedding.clear();
}

void cellData::SaveToFile(ofstream& myfile)
{  
    // bool vertexContainer
    myfile.write((const char*) &vertexContainer, sizeof(bool));

	// vector<weight> influences 
	vector<int> labels;
	vector<float> weights;

	labels.resize(influences.size());
	weights.resize(influences.size());
	for(int i = 0; i< influences.size(); i++)
	{
		labels[i] = influences[i].label;
		weights[i] = influences[i].weightValue;
	}

    // vector<float> weights
    int size = weights.size();
    myfile.write((const char*) &size, sizeof(unsigned int));
	if(size > 0)
		myfile.write((const char*) &weights[0], sizeof(float)*weights.size());

    // vector<int> labels
    size = labels.size();
    myfile.write((const char*) &size, sizeof(unsigned int));
	if(size > 0)
		myfile.write((const char*) &labels[0], sizeof(int)*labels.size());

    // vector<double> embedding
    size = embedding.size();
    myfile.write((const char*) &size, sizeof(unsigned int));
	if(size > 0)
		myfile.write((const char*) &embedding[0], sizeof(double)*embedding.size());

    //int segmentId;
    myfile.write((const char*) &segmentId, sizeof(int));
    
}

void cellData::LoadFromFile(ifstream& myfile)
{
    myfile.read( (char*) &vertexContainer, sizeof(bool) );

    int size;
	vector<int> al;
	vector<float> aw;

	// Weights
	myfile.read( (char*) &size, sizeof(int) );
	if(size > 0)
	{
		aw.resize(size);
		myfile.read( (char*) &aw[0], sizeof(float)*size);
	}
	else aw.clear();

	// labels
    myfile.read( (char*) &size, sizeof(int) );
	if(size > 0)
	{
		al.resize(size);
		myfile.read( (char*) &al[0], sizeof(int)*size);
	}
	else al.clear();

	if(size > 0)
	{
		influences.resize(size);
		for(int i = 0; i< influences.size(); i++)
		{
			influences[i].label = al[i];
			influences[i].weightValue = aw[i];
		}
	}
	else
		influences.clear();

    myfile.read( (char*) &size, sizeof(int) );
	if(size > 0)
	{
		embedding.resize(size);
		myfile.read( (char*) &embedding[0], sizeof(double)*size);
	}
	else embedding.clear();

	myfile.read( (char*) &segmentId, sizeof(int) );

}

/////////////////////////////////////
/////////////   CELL   //////////////

cell3d::cell3d()
{
    setType(UNTYPED);
    data = new cellData();
    frontier = 0;
}

cell3d::cell3d(bool noData)
{
    setType(UNTYPED);
    if(!noData) 
		data = new cellData();
	else
		data = NULL;

    frontier = 0;
}

cell3d::cell3d(int weightsSize)
{
    setType(UNTYPED);
    data = new cellData(weightsSize);
    frontier = 0;
}

cell3d::cell3d(T_cell tipo, int weightsSize)
{
    setType(tipo);
    data = new cellData(weightsSize);
    frontier = 0;
}

cell3d::~cell3d()
{
    data->clear();
    delete data;
    data = NULL;
}

/////////////////////////////////////
////////////   GRID   ///////////////

void grid3d::initBasicData()
{
	cells.clear();
	dimensions = Point3i(0,0,0);
	cellSize = 0;
	weightSize = 0;
	totalCellsCounts = 0;
	borderCellsCounts = 0;
	inCellsCounts = 0;
	outCellsCounts = 0;
	valueRange = 0;

	smoothPropagationRatio = 1;
	worldScale = 1;
	minConfidenceLevel = 0.5;

	kValue = 100;
	alphaValue = 9;

	tradIds.clear();
	bindedSkeletons.clear();

	res = 7;

	fPrintText = NULL;
	fPrintProcessStatus = NULL;
	//loadedFunctions = false;
}

grid3d::grid3d()
{
    initBasicData();
}

void grid3d::initwithNoData(Box3d bounding_, Point3i divisions)
{
	initBasicData();

    // comprobamos que las dimensiones tengan sentido
    assert(divisions.X() != 0 || divisions.Y() != 0 || divisions.Z() != 0);

    bool withData = false;
    // reservamos el espacio necesario para hacer los c√°lculos
    cells.resize(divisions.X());
    for(int i = 0; i< divisions.X(); i++)
    {
        cells[i].resize(divisions.Y());
        for(int j = 0; j< divisions.Y(); j++)
        {
            cells[i][j].resize(divisions.Z());
            for(int k = 0; k< divisions.Z(); k++)
            {
                //cells[i][j][k] = new cell3d(_weightsSize);
                cells[i][j][k] = new cell3d(withData);
            }
        }
    }

    dimensions = divisions;
    bounding = bounding_;

    double cell01 = bounding.DimX()/divisions.X();
    double cell02 = bounding.DimY()/divisions.Y();
    double cell03 = bounding.DimZ()/divisions.Z();

    // Comprobamos de las celdas son regulares.
    double error = 0.0000000001;
    assert(fabs(cell01-cell02)<error && fabs(cell03-cell02)<error);

    cellSize = bounding.DimX()/divisions.X();
}

void grid3d::init(Box3d bounding_, Point3i divisions, int _weightsSize)
{
	initBasicData();

    // comprobamos que las dimensiones tengan sentido
    assert(divisions.X() != 0 || divisions.Y() != 0 || divisions.Z() != 0);

    // reservamos el espacio necesario para hacer los c√°lculos
    cells.resize(divisions.X());
    for(int i = 0; i< divisions.X(); i++)
    {
        cells[i].resize(divisions.Y());
        for(int j = 0; j< divisions.Y(); j++)
        {
            cells[i][j].resize(divisions.Z());
            for(int k = 0; k< divisions.Z(); k++)
            {
                //cells[i][j][k] = new cell3d(_weightsSize);
                cells[i][j][k] = new cell3d(true);
            }
        }
    }

    dimensions = divisions;
    bounding = bounding_;

    double cell01 = bounding.DimX()/divisions.X();
    double cell02 = bounding.DimY()/divisions.Y();
    double cell03 = bounding.DimZ()/divisions.Z();

    // Comprobamos de las celdas son regulares.
    double error = 0.0000000001;
    assert(fabs(cell01-cell02)<error && fabs(cell03-cell02)<error);

    cellSize = bounding.DimX()/divisions.X();
}

grid3d::grid3d(Box3d bounding_, Point3i divisions, int _weightsSize)
{
    // comprobamos que las dimensiones tengan sentido
    assert(divisions.X() != 0 || divisions.Y() != 0 || divisions.Z() != 0);

    // reservamos el espacio necesario para hacer los c√°lculos
    cells.resize(divisions.X());
    for(int i = 0; i< divisions.X(); i++)
    {
        cells[i].resize(divisions.Y());
        for(int j = 0; j< divisions.Y(); j++)
        {
            cells[i][j].resize(divisions.Z());
            for(int k = 0; k< divisions.Z(); k++)
            {
                //cells[i][j][k] = new cell3d(_weightsSize);
                cells[i][j][k] = new cell3d();
            }
        }
    }

    weightSize = 0;
    //weightSize = _weightsSize;

    /*
    for(int i = 0; i< cells.size(); i++)
    {
        for(int j = 0; j<  cells[i].size(); j++)
        {
            for(int k = 0; k< cells[i][j].size(); k++)
            {
                assert(cells[i][j][k]->tipo == UNTYPED);
            }
        }
    }
    */

    dimensions = divisions;
    bounding = bounding_;

    double cell01 = bounding.DimX()/divisions.X();
    double cell02 = bounding.DimY()/divisions.Y();
    double cell03 = bounding.DimZ()/divisions.Z();

    // Comprobamos de las celdas son regulares.
    assert(cell01 == cell02 && cell02 == cell03);

    cellSize = bounding.DimX()/divisions.X();
}

void grid3d::updateStatistics()
{
    totalCellsCounts = 0;
    borderCellsCounts = 0;
    inCellsCounts = 0;
    outCellsCounts = 0;

    int untyped= 0;

    for(unsigned int i = 0; i< cells.size(); i++)
    {
        for(unsigned int j = 0; j< cells[i].size(); j++)
        {
            for(unsigned int k = 0; k< cells[i][j].size(); k++)
            {
                switch(cells[i][j][k]->getType())
                {
                case BOUNDARY:
                    borderCellsCounts++;
                    break;
                case INTERIOR:
                    inCellsCounts++;
                    break;
                case EXTERIOR:
                    outCellsCounts++;
                    break;
                default:
                    untyped++;
                    break;
                }
                totalCellsCounts++;
            }
        }
    }
}

void grid3d::clear()
{
    // Celdas del grid
    for(unsigned int i = 0; i< cells.size(); i++)
    {
        for(unsigned int j = 0; j< cells[i].size(); j++)
        {
            for(unsigned int k = 0; k< cells[i][j].size(); k++)
            {
                delete cells[i][j][k];
                cells[i][j][k] = NULL;
            }
            cells[i][j].clear();
        }
        cells[i].clear();
    }
    cells.clear();

    dimensions = Point3i(0,0,0);
    bounding.SetNull();
    weightSize = 0;
    cellSize = 0;
}

int grid3d::fillFromCorners()
{
    vector<Point3i> listForProcess;

    int cellsCount = 0;

    // Encolamos los 8 extremos.
    listForProcess.push_back(Point3i(0,0,0));
    listForProcess.push_back(Point3i(0,dimensions.Y()-1,0));
    listForProcess.push_back(Point3i(0,0,dimensions.Z()-1));
    listForProcess.push_back(Point3i(0,dimensions.Y()-1,dimensions.Z()-1));
    listForProcess.push_back(Point3i(dimensions.X()-1,0,0));
    listForProcess.push_back(Point3i(dimensions.X()-1,dimensions.Y()-1,0));
    listForProcess.push_back(Point3i(dimensions.X()-1,0,dimensions.Z()-1));
    listForProcess.push_back(Point3i(dimensions.X()-1,dimensions.Y()-1,dimensions.Z()-1));

    // Los posibles vecinos de expansion
    vector<Point3i> neighbours;
    neighbours.push_back(Point3i(0,0,-1));
    neighbours.push_back(Point3i(0,0,1));
    neighbours.push_back(Point3i(0,-1,0));
    neighbours.push_back(Point3i(0,1,0));
    neighbours.push_back(Point3i(-1,0,0));
    neighbours.push_back(Point3i(1,0,0));

    // Hasta que se vacie la cola recorremos los cubos vecinos.
    while(!listForProcess.empty())
    {
        Point3i pos = listForProcess.back(); listForProcess.pop_back();

        if(cells[pos.X()][pos.Y()][pos.Z()]->getType() == UNTYPED)
        {
            cells[pos.X()][pos.Y()][pos.Z()]->setType(EXTERIOR);
            cellsCount++;
        }
        else
            continue;

        for(unsigned int l = 0; l< neighbours.size(); l++)
        {
            int i = neighbours[l].X();
            int j = neighbours[l].Y();
            int k = neighbours[l].Z();

            if(pos.X() + i >= 0 && pos.Y() + j >= 0 && pos.Z() + k >= 0 &&
               pos.X() + i < dimensions.X() && pos.Y() + j < dimensions.Y() && pos.Z() + k  < dimensions.Z())
            {
                if(cells[pos.X() + i][pos.Y() + j][pos.Z() + k]->getType() == UNTYPED)
                    listForProcess.push_back(Point3i(pos.X() + i,pos.Y() + j,pos.Z() + k));
            }
        }
    }

    return cellsCount;
}

int grid3d::fillInside()
{
    int cellsCount = 0;
    for(int i = 0; i< dimensions.X(); i++)
    {
        for(int j = 0; j< dimensions.Y(); j++)
        {
            for(int k = 0; k< dimensions.Z(); k++)
            {
                if(cells[i][j][k]->getType() == UNTYPED)
                {
                    cells[i][j][k]->setType(INTERIOR);
                    cellsCount++;
                }
            }
        }
    }

    return cellsCount;
}

#define TERMINATE_CRITERION 0.00001


void grid3d::cleanZeroInfluences()
{
	for(int i = 0; i< dimensions.X(); i++)
	{
		for(int j = 0; j< dimensions.Y(); j++)
		{
			for(int k = 0; k< dimensions.Z(); k++)
			{
				if(cells[i][j][k]->getType() != BOUNDARY) continue;

				cellData* cellD = cells[i][j][k]->data;
					
				// Eliminamos los pesos igual a 0
				cellD->auxInfluences.clear();
				for(int infl = 0; infl < cellD->influences.size(); infl++)
				{
					if(cellD->influences[infl].weightValue != 0)
						cellD->auxInfluences.push_back(cellD->influences[infl]);
				}

				cellD->influences.clear();
				for(int infl = 0; infl < cellD->auxInfluences.size(); infl++)
				{
					cellD->influences.push_back(cellD->auxInfluences[infl]);
				}
			}
		}
	}

}

// Importante: tiene que estar bien inicializado el dominio
// y los vectores de influencias auxiliares.
void grid3d::normalizeWeightsByDomain()
{
	for(int i = 0; i< dimensions.X(); i++)
    {
        for(int j = 0; j< dimensions.Y(); j++)
        {
            for(int k = 0; k< dimensions.Z(); k++)
            {
				if(cells[i][j][k]->getType() != BOUNDARY) 
					continue;

				cellData* cellD = cells[i][j][k]->data;

				float childGain = 0;
				for(int infl = 0; infl< cellD->auxInfluences.size(); infl++)
				{

					if(cellD->auxInfluences[infl].weightValue < 0 ||
						cellD->auxInfluences[infl].weightValue > 1)
						printf("hay algun problema en la asignacion de pesos.\n");
					childGain += cellD->auxInfluences[infl].weightValue;
				}

				if(childGain == 0) 
				{
					cellD->auxInfluences.clear();
					continue;
				}

				if(cellD->domain > childGain)
				{
					//printf("\n\nEn principio aquÌ no entra, porque el padre tambiÈn juega.\n\n"); fflush(0);
					if(cellD->domainId >= 0)
					{						
						// Obtener la influencia del padre y quitar lo que tocarÌa.
						int fatherId = findWeight(cellD->influences, cellD->domainId);
						if(fatherId >= 0)
						{
							cellD->influences[fatherId].weightValue = cellD->domain - childGain;
						}
					}

					for(int infl = 0; infl< cellD->auxInfluences.size(); infl++)
					{
						int l = cellD->auxInfluences[infl].label;
						float w = cellD->auxInfluences[infl].weightValue;
						cellD->influences.push_back(weight(l,w));
					}					
				}
				else
				{
					// Eliminamos el peso del padre, porque lo repartiremos.
					if(cellD->domainId >= 0)
					{				
						int fatherId = findWeight(cellD->influences, cellD->domainId);
						if(fatherId >= 0)
						{
							if(cellD->influences[fatherId].weightValue != cellD->domain)
							{
								printf("Hay un problema de inicializacion del dominio...\n");
								fflush(0);
							}
							
							cellD->influences[fatherId].weightValue = 0;

							// Quitamos la influencia del padre
							//vector<weight> tempWeights;
							//for(int tw = 0; tw < cellD->influences.size(); tw++)
							//{
							//	if(tw == fatherId) continue;
							//	tempWeights.push_back(cellD->influences[tw]);
							//}
							//cellD->influences.clear();

							//for(int tw = 0; tw < tempWeights.size(); tw++)
							//{
							//	cellD->influences.push_back(tempWeights[tw]);
							//}
						}
					}

					for(int infl = 0; infl< cellD->auxInfluences.size(); infl++)
					{
						int l = cellD->auxInfluences[infl].label;
						float w = (cellD->auxInfluences[infl].weightValue/childGain)*cellD->domain;

						float a = w;
						if(a != w)
						{
							int problema = 1;
							printf("Tenemos un problema curioso.\n");
						}

						cellD->influences.push_back(weight(l,w));
					}	
				}

				cellD->auxInfluences.clear();
            }
        }
    }

}

void grid3d::normalizeWeights()
{
	/*
    if(DEBUG)
    {
        float maxValue = -9999, minValue = 9999;
        for(int i = 0; i< dimensions.X(); i++)
        {
            for(int j = 0; j< dimensions.Y(); j++)
            {
                for(int k = 0; k< dimensions.Z(); k++)
                {
                    if(cells[i][j][k]->getType() == INTERIOR)
                    {
                        if(cells[i][j][k]->data->vertexContainer)
                            continue;

                        for(int w = 0; w < weightSize; w++)
                        {
                            maxValue = max(cells[i][j][k]->data->weights[w], maxValue);
                            minValue = min(cells[i][j][k]->data->weights[w], minValue);
                        }
                    }
                }
            }
        }

        printf("MinValue: %f; MaxValue: %f\n", minValue, maxValue); fflush(0);
    }

    for(int i = 0; i< dimensions.X(); i++)
    {
        for(int j = 0; j< dimensions.Y(); j++)
        {
            for(int k = 0; k< dimensions.Z(); k++)
            {
                if(cells[i][j][k]->getType() == INTERIOR)
                {
                    if(cells[i][j][k]->data->vertexContainer)
                        continue;

                    double totalValue = 0;
                    for(int w = 0; w < weightSize; w++)
                    {
                        totalValue += cells[i][j][k]->data->weights[w];
                    }

                    if(totalValue != 0)
                    {
                        for(int w = 0; w < weightSize; w++)
                        {
                            cells[i][j][k]->data->weights[w] = cells[i][j][k]->data->weights[w]/totalValue;
                        }
                    }
                }
            }
        }
    }

    if(DEBUG)
    {
        float maxValue = -9999, minValue = 9999;
        for(int i = 0; i< dimensions.X(); i++)
        {
            for(int j = 0; j< dimensions.Y(); j++)
            {
                for(int k = 0; k< dimensions.Z(); k++)
                {
                    if(cells[i][j][k]->getType() == INTERIOR)
                    {
                        if(cells[i][j][k]->data->vertexContainer)
                            continue;

                        for(int w = 0; w < weightSize; w++)
                        {
                            maxValue = max(cells[i][j][k]->data->weights[w], maxValue);
                            minValue = min(cells[i][j][k]->data->weights[w], minValue);
                        }
                    }
                }
            }
        }

        printf("MinValue: %f; MaxValue: %f; weightSize:%d\n", minValue, maxValue, weightSize); fflush(0);
    }
	*/
}

void grid3d::expandWeights()
{

	assert(false); // Disabled for flux verification.

	/*
    // Inicializamos los posibles vecinos en un array para
    // Simplificar el c√≥digo -> 6-conectado
    vector<Point3i> positions;
    positions.resize(6);
    positions[0] = Point3i(1,0,0);
    positions[1] = Point3i(-1,0,0);
    positions[2] = Point3i(0,1,0);
    positions[3] = Point3i(0,-1,0);
    positions[4] = Point3i(0,0,1);
    positions[5] = Point3i(0,0,-1);

    float iterationVariation = 9999;
    int iterations = 0;

    vector< cell3d* > processCells;
    for(int i = 0; i< dimensions.X(); i++)
    {
        for(int j = 0; j< dimensions.Y(); j++)
        {
            for(int k = 0; k< dimensions.Z(); k++)
            {
                if(cells[i][j][k]->getType() != EXTERIOR )
                {
                    processCells.push_back(cells[i][j][k]);
                    cells[i][j][k]->data->pos = Point3i(i,j,k);
                    cells[i][j][k]->changed = false;
                }
            }
        }
    }

    printf("\nProcesar con %d celdas interiores o borde\n", processCells.size());fflush(0);

    vector<bool> expandedWeight;
    vector<float> interationVariationArray;
    expandedWeight.resize(weightSize, false);
    interationVariationArray.resize(weightSize, 0);

    while(iterationVariation >= TERMINATE_CRITERION)
    {
        for(unsigned int cellCount = 0; cellCount< processCells.size(); cellCount++)
        {

            cell3d* currentCell = processCells[cellCount];
            int i = currentCell->data->pos.X();
            int j = currentCell->data->pos.Y();
            int k = currentCell->data->pos.Z();

            if(cells[i][j][k]->getType() == INTERIOR || cells[i][j][k]->getType() == BOUNDARY)
            {

                if(cells[i][j][k]->data->vertexContainer) continue;

                // limpiamos del anterior c√°lculo.
                cells[i][j][k]->data->auxWeights.clear();
                cells[i][j][k]->data->auxWeights.resize(cells[i][j][k]->data->weights.size());

                for(int w = 0; w < weightSize; w++)
                {
                    // Si ese peso ya se ha expandido suficiente, lo dejamos.
                    //if(expandedWeight[w]) continue;

                    // Iniciamos el valor para el calculo de la media.
                    float tempValue = 0;
                    //cells[i][j][k]->auxWeights[w] = 0;

                    float inPos = 0;
                    // Para cada uno de los vecinos 6-conectado.
                    for(unsigned int p = 0; p < positions.size(); p++)
                    {
                        int newI = i+positions[p].X();
                        int newJ = j+positions[p].Y();
                        int newK = k+positions[p].Z();

                        // Comprobamos que no se salga del grid.
                        if(newI <0 || newI >= dimensions.X()) continue;
                        if(newJ <0 || newJ >= dimensions.Y()) continue;
                        if(newK <0 || newK >= dimensions.Z()) continue;

                        if(cells[newI][newJ][newK]->getType() == EXTERIOR) continue;

                        // Acumulamos el valor de los vecinos.
                        float v1 = cells[newI][newJ][newK]->data->weights[w];
                        tempValue += v1;
                        inPos++;
                        //someChanged |= cells[newI][newJ][newK]->weights[w] > 0 || cells[i][j][k]->weights[w] > 0;
                    }

                    // Hacemos la media
                    if(inPos > 0)
                    {
                        tempValue = tempValue / inPos;

                        // Si ha habido cambio...vemos cuanto.
                        if(tempValue > 0)
                        {
                            cells[i][j][k]->data->auxWeights[w] = tempValue;
                        }
                    }
                }
            }
        }


        // Actualizamos los valores del gris con los que hemos calculado.
        int counter = 0;
        float meanVar = 0;

        for(unsigned int cellCount = 0; cellCount< processCells.size(); cellCount++)
        {
            cell3d* currentCell = processCells[cellCount];
            int i = currentCell->data->pos.X();
            int j = currentCell->data->pos.Y();
            int k = currentCell->data->pos.Z();

            if(cells[i][j][k]->getType() == INTERIOR || cells[i][j][k]->getType() == BOUNDARY)
            {

                if(cells[i][j][k]->data->vertexContainer)
                    continue;

                float diference = 0;
                for(int w = 0; w < weightSize; w++)
                {
                    //if(expandedWeight[w]) continue;

                    diference = fabs(cells[i][j][k]->data->auxWeights[w]- cells[i][j][k]->data->weights[w]);
                    interationVariationArray[w] += diference;
                    meanVar += diference;

                    if(cells[i][j][k]->data->auxWeights[w] > 0)
                        cells[i][j][k]->data->weights[w] =  cells[i][j][k]->data->auxWeights[w] ;
                }
                counter++;
            }
        }

        int expanded = 0;
        if(counter > 0)
        {
            for(int cc = 0; cc< weightSize; cc++)
            {
                if(expandedWeight[cc])
                {
                    expanded++;
                    //continue;
                }

                interationVariationArray[cc] = interationVariationArray[cc]/counter;

                if(interationVariationArray[cc] < TERMINATE_CRITERION) expandedWeight[cc] = true;
            }
            meanVar = meanVar/(counter*weightSize);
        }
        else
            printf("El contador está a cero... no ha habido nigún cambio.");

        // Nos quedamos con la media mayor. Se irá iterando hasta que todas las medias estén por debajo, pero no se modificará si ya lo está.
        //iterationVariation = 0;

       iterationVariation = min(meanVar, iterationVariation);

        printf("Iteracion: %d, con valor: %f (%d pesos expandidos).\n", iterations, iterationVariation, expanded);fflush(0);
        iterations++;
    }

	*/
}

int grid3d::typeCells(MyMesh& mesh)
{
    // Comprobar que la caja contenedora de la maya est√° contenida dentro del grid.
    //assert(mesh.bbox.min > bounding.min);
    //assert(mesh.bbox.max < bounding.max);

    int TypedCells = 0;

    // recorremos todas las caras y las anadimos al grid -> BOUNDARIES
    for(MyMesh::FaceIterator fi = mesh.face.begin(); fi!=mesh.face.end(); ++fi )
        TypedCells += typeCells(*fi);

    // Marcamos lo que es externo.
    fillFromCorners();

    // Marcamos lo que es interno.
    fillInside();

    return TypedCells;
}

Point3i grid3d::cellId(Point3d pt)
{
    Point3d aux = pt - bounding.min;
    int x = (int)round(aux.X()/cellSize);
    int y = (int)round(aux.Y()/cellSize);
    int z = (int)round(aux.Z()/cellSize);
    return Point3i(x,y,z);
}

Point3d grid3d::cellCenter(int i,int j,int k)
{
    Point3d aux = bounding.min;
    aux += Point3d(cellSize*(i+0.5),cellSize*(j+0.5),cellSize*(k+0.5));
    return aux;
}

int grid3d::typeCells(MyFace& face)
{
    // A. Anadimos los vertices.
    Point3i cell[3];
    Box3i boundingCells;

    int boundaryCells = 0;

    // recogemos los vertices y los marcamos.
    for(int i = 0; i<3; i++)
    {
        cell[i] = cellId(face.P(i)); // Obtenemos la celda en la que cae el v√©rtice
        boundingCells.Add(cell[i]);

        if(cells[cell[i].X()][cell[i].Y()][cell[i].Z()]->getType() != BOUNDARY)
        {
            cells[cell[i].X()][cell[i].Y()][cell[i].Z()]->setType(BOUNDARY);
            boundaryCells++;
        }

		if(cells[cell[i].X()][cell[i].Y()][cell[i].Z()]->data == NULL)
			cells[cell[i].X()][cell[i].Y()][cell[i].Z()]->data = new cellData();

        cells[cell[i].X()][cell[i].Y()][cell[i].Z()]->data->vertexContainer = true;
        //cells[cell[i].X()][cell[i].Y()][cell[i].Z()]->weights[face.V(i)->IMark()] = 1.0;
    }

    // Ahora recorremos celda por celda y vamos pintando el contorno.

    float processCellSize = cellSize/3;

    // 3D Rendering de arestas
    // cellSize -> lado de cada cubo.
    // ponemos el valor para las arestas también.
    for(int k = 0; k<3; k++)
    {
        Point3d v = (face.V((k+1)%3)->P()-face.V(k)->P());

        //int idvert1 = face.V(k)->IMark();
        //int idvert2 = face.V((k+1)%3)->IMark();
        Point3d v1 = face.V(k)->P();
        //Point3d v2 = face.V((k+1)%3)->P();
        //float edgeLength = (v2-v1).Norm();

        int divisions = (int)floor(v.Norm()/processCellSize);
        Point3d vDir = v/v.Norm();
        for(int i = 0; i< divisions ; i++)
        {
            Point3d intPoint = vDir*i*processCellSize + v1;
            Point3i cell = cellId(intPoint);

            if(dimensions.X() <= cell.X() || dimensions.Y() <= cell.Y() || dimensions.Z() <= cell.Z() ||
               0 > cell.X() || 0 > cell.Y() || 0 > cell.Z()      )
            {
                printf("Tenemos un punto fuera?? (%d, %d, %d)\n", cell.X(), cell.Y(), cell.Z());
            }
            else
            {
				cell3d* cellT = cells[cell.X()][cell.Y()][cell.Z()];

                if(cellT->getType() != BOUNDARY)
                {
					cellT->setType(BOUNDARY);
					if(cellT->data == NULL) cellT->data = new cellData();
                }
            }
            boundaryCells++;
        }
    }

    // buscamos la aresta mas larga.
    int largeIdx = 0;
    Point3d v = (face.V((largeIdx+1)%3)->P()-face.V(largeIdx)->P());
    float edgeLong = v.Norm();
    for(int k = 1; k<3; k++)
    {
        v = Point3d(face.V((k+1)%3)->P()-face.V(k)->P());
        if(edgeLong < v.Norm())
        {
            largeIdx = k;
            edgeLong = v.Norm();
        }
    }

    Point3d v1 = Point3d(face.V((largeIdx+1)%3)->P()-face.V(largeIdx)->P());
    Point3d v2 = Point3d(face.V((largeIdx+2)%3)->P()-face.V(largeIdx)->P());
    Point3d v3 = Point3d(face.V((largeIdx+2)%3)->P()-face.V((largeIdx+1)%3)->P());

    Point3d aux = v1^v2;
    Point3d normal = aux^v1;

    float v1Norm = v1.Norm();
    float v2Norm = v2.Norm();
    float v3Norm = v3.Norm();
    float normalNorm = normal.Norm();

    Point3d v1Dir = v1/v1Norm;
    Point3d v2Dir = v2/v2Norm;
    Point3d v3Dir = v3/v3Norm;
    Point3d normalDir = normal/normalNorm;

    Point3d edgeCenter = v1Dir*(v1Dir*v2) + face.V(largeIdx)->P();
    int div1 = (int)ceil((face.V(largeIdx)->P()-edgeCenter).Norm()/processCellSize);
    int div2 = (int)ceil((face.V((largeIdx+1)%3)->P()-edgeCenter).Norm()/processCellSize);

    for(int i = 1; i< div1 ; i++) // Saltamos el 0 porque es el mismo vertice.
    {
        Point3d minPt = v1Dir*i*processCellSize + face.V(largeIdx)->P();
        Point3d maxPt = v2Dir*((float)i/(float)div1)*v2Norm + face.V(largeIdx)->P(); // Suponemos que al ser triangulo rectangulo mantiene proporciones.

        Point3d line = maxPt-minPt;
        int Ydivs = (int)floor(line.Norm()/processCellSize);

        for(int j = 1; j< Ydivs ; j++) // Saltamos el 0 porque es el mismo v√©rtice.
        {
            Point3d intPoint = normalDir*j*processCellSize + minPt;
            Point3i cell = cellId(intPoint);

            if(dimensions.X() <= cell.X() || dimensions.Y() <= cell.Y() || dimensions.Z() <= cell.Z() ||
               0 > cell.X() || 0 > cell.Y() || 0 > cell.Z()      )
            {
                printf("Tenemos un punto fuera?? (%d, %d, %d)\n", cell.X(), cell.Y(), cell.Z());
            }
            else
            {
				if(cells[cell.X()][cell.Y()][cell.Z()]->data == NULL)
					cells[cell.X()][cell.Y()][cell.Z()]->data = new cellData();

                cells[cell.X()][cell.Y()][cell.Z()]->setType(BOUNDARY);
            }

            boundaryCells++;
        }
    }


    for(int i = 1; i< div2 ; i++) // Saltamos el 0 porque es el mismo vertice.
    {
        Point3d minPt = -v1Dir*i*processCellSize + face.V((largeIdx+1)%3)->P();
        Point3d maxPt = v3Dir*((float)i/(float)div2)*v3Norm + face.V((largeIdx+1)%3)->P(); // Suponemos que al ser tri√°ngulo rect√°ngulo mantiene proporciones.

        Point3d line = maxPt-minPt;
        int Ydivs = (int)floor(line.Norm()/processCellSize);

        for(int j = 1; j< Ydivs ; j++) // Saltamos el 0 porque es el mismo v√©rtice.
        {
            Point3d intPoint = normalDir*j*processCellSize + minPt;
            Point3i cell = cellId(intPoint);

            if(dimensions.X() <= cell.X() || dimensions.Y() <= cell.Y() || dimensions.Z() <= cell.Z() ||
               0 > cell.X() || 0 > cell.Y() || 0 > cell.Z()      )
            {
                printf("Tenemos un punto fuera?? (%d, %d, %d)\n", cell.X(), cell.Y(), cell.Z());
            }
            else
			{
				if(cells[cell.X()][cell.Y()][cell.Z()]->data == NULL)
					cells[cell.X()][cell.Y()][cell.Z()]->data = new cellData();

                cells[cell.X()][cell.Y()][cell.Z()]->setType(BOUNDARY);
			}

            boundaryCells++;
        }
    }

    
    // Podemos ampliar el contorno para estabilizar más el cálculo.
    /*
	for(int i = 0; i< cells.size(); i++)
    {
        for(int j = 0; j< cells[i].size(); j++)
        {
            for(int k = 0; k< cells[i][j].size(); k++)
            {
                if(cells[i][j][k]->tipo == BOUNDARY && !cells[i][j][k]->changed)
                {

                    for(int l = -1; l< 2; l++)
                    {
                        for(int m = -1; m< 2; m++)
                        {
                            for(int n = -1; n< 2; n++)
                            {
                                if(l == 0 && m == 0 && n == 0) continue;


                                if(l+i < 0 || m+j < 0 || n+k < 0 || dimensions.X() <= l+i || dimensions.Y() <= m+j || dimensions.Z() <= n+k )  continue;

                                if(cells[i][j][k]->tipo == BOUNDARY) continue;

                                if(cells[i+l][j+m][k+n])
                                {
                                    cells[i+l][j+m][k+n]->tipo = BOUNDARY;
                                    cells[i][j][k]->changed = true;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    */

     return boundaryCells;
}


// Guardamos en binario el grid entero, para no tener que recalcular cada vez
void grid3d::LoadGridFromFile(string sFileName)
{
    ifstream myfile;
	const char* charFileName = sFileName.c_str();
    myfile.open(charFileName, ios::in |ios::binary);
    if (myfile.is_open())
    {
        /* DATOS A GUARDAR
        vector< vector< vector< cell3d* > > > cells;
        Point3i dimensions;
        Box3d bounding;
        int weightSize;
        float cellSize;
        */

        // Datos generales
        myfile.read((char*)  &res, sizeof(int) );
        myfile.read( (char*) &dimensions[0], sizeof(int)*3 );
        myfile.read( (char*) &cellSize, sizeof(float) );
        myfile.read( (char*) &weightSize, sizeof(int));

        Point3d minPt,maxPt ;
        myfile.read( (char*) &minPt, sizeof(double)*3);
        myfile.read( (char*) &maxPt, sizeof(double)*3);
        bounding = Box3d(minPt, maxPt);

        myfile.read( (char*) &valueRange, sizeof(float));

        cells.resize(dimensions[0]);

        //Datos de cada una de las celdas.
        for(unsigned int i = 0; i< cells.size(); i++)
        {
            cells[i].resize(dimensions[1]);
            for(unsigned int j = 0; j< cells[i].size(); j++)
            {
                cells[i][j].resize(dimensions[2]);
                for(unsigned int k = 0; k< cells[i][j].size(); k++)
                {
                    cells[i][j][k] = new cell3d(true);
                    cells[i][j][k]->LoadFromFile(myfile);
                }
            }
        }

    }
    myfile.close();

}

// Cargamos de disco el grid entero.
void grid3d::SaveGridToFile(string sFileName)
{
    ofstream myfile;
    myfile.open(sFileName.c_str(), ios::binary);
    if (myfile.is_open())
    {
        /* DATOS A GUARDAR
            vector< vector< vector< cell3d* > > > cells;
            Point3i dimensions;
            Box3d bounding;
            int weightSize;
            float cellSize;
        */

        // Datos generales
        myfile.write((const char*) &res, sizeof(int));
        myfile.write((const char*) &dimensions, sizeof(int)*3);
        myfile.write((const char*) &cellSize, sizeof(float));
        myfile.write((const char*) &weightSize, sizeof(int));
        myfile.write((const char*) &bounding.min, sizeof(double)*3);
        myfile.write((const char*) &bounding.max, sizeof(double)*3);
        myfile.write((const char*) &valueRange, sizeof(float));


        //Datos de cada una de las celdas.
        for(unsigned int i = 0; i< cells.size(); i++)
        {
            for(unsigned int j = 0; j< cells[i].size(); j++)
            {
                for(unsigned int k = 0; k< cells[i][j].size(); k++)
                {
                    cells[i][j][k]->SaveToFile(myfile);
                }
            }
        }

    }
    myfile.close();


}

void cell3d::SaveToFile(ofstream& myfile)
{
    /* DATOS A GUARDAR
    // tipo de la celda
    T_cell tipo;

    // vertexContainer
    bool vertexContainer;

    // valores de pesos de cada v√©rtice de la caja envolvente
    vector<float> weights;
    */

    int auxType = getType();
    myfile.write((const char*) &auxType, sizeof(int));

	// Solo guardamos si es boundary
	if(getType() == BOUNDARY)
		data->SaveToFile(myfile);
}

void cell3d::LoadFromFile(ifstream& myfile)
{
    int tipoAux;
    myfile.read( (char*) &tipoAux, sizeof(int) );
    setType((T_cell)tipoAux);

	// Solo cargamos si es boundary
	if(getType() == BOUNDARY)
	{
		if(data == NULL) 
			data = new cellData();

		data->LoadFromFile(myfile);
	}
}
