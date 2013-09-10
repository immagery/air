#include "Segmentation.h"

#include "..\utils\util.h"
#include "..\DataStructures\InteriorDistancesData.h"
#include "mvc_interiorDistances.h"

#include "..\DataStructures\skeleton.h"
#include "..\render\gridRender.h"
#include "..\DataStructures\modelo.h"
#include "..\DataStructures\scene.h"

#include <cmath>
using namespace std;

#define MAX_LENGTH 99999999

#define FIRST_ITERATION -1

#define MIN_CONFIDENCE_LEVEL 1.0

#define PROPAGATION_UNIT 10.0

#define VERBOSE false

#define bCutRegions false
#define bPropagate true
#define bNormalizeByDomain true

// Euclidean Point Distance
float interiorDistance(Point3d& cellCtr,
                       Point3d& nodePt)
{
    return (cellCtr-nodePt).Norm();
}

// Interior Distances test with euclidean points
bool ownerDistIsLower(Point3d ptOwner,
                      Point3d intruder,
                      Point3d cell)
{
    return ( interiorDistance(ptOwner,cell) <= interiorDistance(cell,intruder) );
}

// Interior Distances test with embedded points
bool ownerDistIsLower(vector<double> ptOwner,
                      vector<double> intruder,
                      vector<double> cell)
{
    return ( distancesFromEbeddedPoints(ptOwner,cell) <= distancesFromEbeddedPoints(cell,intruder) );
}

// Precomputa las distancias de una celda a sus vecinas, dado un tamaño de celda.
void precomputeInterCellDistances(vector< vector< vector<float> > >& displacementMatrix,
                                  float cellSize)
{
    displacementMatrix.resize(3);
    for(int i = 0; i< 3; i++)
    {
        displacementMatrix[i].resize(3);
        for(int j = 0; j< 3; j++)
        {
            displacementMatrix[i][j].resize(3);
        }
    }

    // There are 3 posible distances:
    // 1. - cellSize: side by side
    // 2. - sqrt2: 2D diagonal
    // 3. - sqrt3: 3D diagonal
    double sqrt2 = sqrt(2*cellSize*cellSize);
    double sqrt3 = sqrt(3*cellSize*cellSize);

    displacementMatrix[1][1][2] = cellSize;
    displacementMatrix[1][1][0] = cellSize;
    displacementMatrix[1][2][1] = cellSize;
    displacementMatrix[1][0][1] = cellSize;
    displacementMatrix[2][1][1] = cellSize;
    displacementMatrix[0][1][1] = cellSize;

    displacementMatrix[2][2][0] = sqrt3;
    displacementMatrix[2][0][2] = sqrt3;
    displacementMatrix[2][0][0] = sqrt3;
    displacementMatrix[2][2][2] = sqrt3;
    displacementMatrix[0][2][0] = sqrt3;
    displacementMatrix[0][0][2] = sqrt3;
    displacementMatrix[0][0][0] = sqrt3;
    displacementMatrix[0][2][2] = sqrt3;

    displacementMatrix[1][2][0] = sqrt2;
    displacementMatrix[1][0][2] = sqrt2;
    displacementMatrix[1][0][0] = sqrt2;
    displacementMatrix[1][2][2] = sqrt2;
    displacementMatrix[2][1][0] = sqrt2;
    displacementMatrix[0][1][2] = sqrt2;
    displacementMatrix[0][1][0] = sqrt2;
    displacementMatrix[2][1][2] = sqrt2;
    displacementMatrix[2][0][1] = sqrt2;
    displacementMatrix[0][2][1] = sqrt2;
    displacementMatrix[0][0][1] = sqrt2;
    displacementMatrix[2][2][1] = sqrt2;
}


////////////////////////////////////////////////////
/////////////// SEGMENTATION PROCESS ///////////////
////////////////////////////////////////////////////

// Updates a cell advancing front. It presuposes that is needded to propagate from this cell.
//
// - grid: where de front is propagated
// - cell: the current front cell propagated
// - idFront: current front propagated ID
// - secondFront: the next front for being propagated
// - it: the current propagation iteration (ensures that a cell is not processed  twice)
bool updateFront(grid3d& grid,Point3i cell, int idFront, vector<Point3i>& secondfront, int it)
{
    bool updated = false;
 
	/*
	for(int i = -1; i<= 1; i++)
    {
        for(int j = -1; j<=1; j++)
        {
            for(int k = -1; k<=1; k++)
            {
                if(i==0 && j== 0 && k== 0) continue;

                Point3i newCell(cell.X()+i,cell.Y()+j,cell.Z()+k);

                // comprobación de estar dentro del grid... en principio siempre se da.
                assert(newCell.X() >= 0 && newCell.X() < grid.dimensions.X());
                assert(newCell.Y() >= 0 && newCell.Y() < grid.dimensions.Y());
                assert(newCell.Z() >= 0 && newCell.Z() < grid.dimensions.Z());

                // Mientra sea exterior y no sea el mismo label
                if(grid.cells[newCell.X()][newCell.Y()][newCell.Z()]->getType() != EXTERIOR)
                {

                    if(grid.cells[newCell.X()][newCell.Y()][newCell.Z()]->data->labels.size() == 0)
                    {
                        // Si ya lo hemos encolado... no lo volvemos a poner.
                        if(grid.cells[newCell.X()][newCell.Y()][newCell.Z()]->data->itPass == it)
                            continue;

                        secondfront.push_back(newCell);
                        grid.cells[newCell.X()][newCell.Y()][newCell.Z()]->data->itPass = it;
                        updated = true;
                    }
                    else if(grid.cells[newCell.X()][newCell.Y()][newCell.Z()]->data->ownerLabel != idFront)
                    {
                        secondfront.push_back(newCell);
                        grid.cells[newCell.X()][newCell.Y()][newCell.Z()]->data->itPass = it;
                        updated = true;
                    }
                }
            }
        }
    }
	*/
    return updated;
}

// Front propagation
//
// - grid: where de front is propagated
// - front/secondFront: fronts propagated. Two for each node (primary and secondary), that is because
//                      we want to expand the nodes iteratively for faster convergence.
// - embeddedPoints: precalculated embedding for node points.
bool frontPropagationWithEmbedding(grid3d& grid,
                      vector< vector<Point3i> >& front,
                      vector< vector<Point3i> >& secondFront,
                      vector< vector<double> >& embeddedPoints,
                      int it)
{
    bool bGlobalExpansion = false;

    for(unsigned int idFront = 0; idFront< front.size(); idFront++)
    {
        while(front[idFront].size() > 0)
        {
            // Cogemos la primera celda
            Point3i cell = front[idFront].back();
            front[idFront].pop_back();

            // nos hemos salido del modelo.
            if(grid.cells[cell.X()][cell.Y()][cell.Z()]->getType() == EXTERIOR)
                continue;

            // No se ha asignado todavía ningún label
            if(grid.cells[cell.X()][cell.Y()][cell.Z()]->data->ownerLabel < 0)
            {
                grid.cells[cell.X()][cell.Y()][cell.Z()]->data->ownerLabel = idFront;
                bool expanded = updateFront(grid, cell, idFront, secondFront[idFront], it);
                bGlobalExpansion |= expanded;
            }
            else // Ya estaba asignado
            {
                // Evita encolar celdas del mismo frente.
                if(grid.cells[cell.X()][cell.Y()][cell.Z()]->data->ownerLabel == (int)idFront)
                    continue;

                int idOwnerFront = grid.cells[cell.X()][cell.Y()][cell.Z()]->data->ownerLabel;

                // EUCLIDEAN: if(!ownerDistIsLower(points[idOwnerFront], points[idFront], grid.cellCenter(cell.X(),cell.Y(),cell.Z())))

                //vector<double> cellEmbeddedPt;
                //Point3d cellCent = grid.cellCenter(cell.X(),cell.Y(),cell.Z());
                //mvcEmbedPoint(cellCent, cellEmbeddedPt, embedding, mesh);

                if(!ownerDistIsLower(embeddedPoints[idOwnerFront], embeddedPoints[idFront], grid.cells[cell.X()][cell.Y()][cell.Z()]->data->embedding))
                {
                   grid.cells[cell.X()][cell.Y()][cell.Z()]->data->ownerLabel = idFront; // Asignamos el nuevo label
                   bool expanded = updateFront(grid, cell, idFront, secondFront[idFront], it);

                   bGlobalExpansion |= expanded;
                }
            }
        }
    }

    return bGlobalExpansion;
}

void traducePartialSegmentation(grid3d& grid, map<int, int>& traductionTable)
{
    for(unsigned int i = 0; i< grid.cells.size(); i++)
    {
        for(unsigned int j = 0; j< grid.cells[i].size(); j++)
        {
            for(unsigned int k = 0; k< grid.cells[i][j].size(); k++)
            {
				if(!(grid.cells[i][j][k]->data) || grid.cells[i][j][k]->getType() != BOUNDARY) continue;

                cell3d* cell = grid.cells[i][j][k];
                if(cell->data->segmentId>=0)
                {
                    cell->data->ownerLabel = traductionTable[cell->data->segmentId];
                }
				else
				{
					printf("Hay algunas celdas visitadas que no tienen bien asignado el segmentId.(traduceParcialSegmentation)");
				}
            }
        }
    }
}

void traduceSegmentation(grid3d& grid, vector<int>& traductionTable)
{
    for(unsigned int i = 0; i< grid.cells.size(); i++)
    {
        for(unsigned int j = 0; j< grid.cells[i].size(); j++)
        {
            for(unsigned int k = 0; k< grid.cells[i][j].size(); k++)
            {
                cell3d* cell = grid.cells[i][j][k];
                if(cell->data && cell->data->ownerLabel>=0)
                {
                    cell->data->ownerLabel = traductionTable[cell->data->ownerLabel];
                }
            }
        }
    }
}


// Segment volume with labels constructing a voronoi diagram
int segmentVolumeWithEmbedding( grid3d& grid,
                                vector< DefNode >& points,
                                vector< vector<double> >& embeddedPoints,
                                bool onlyBorders)
{
	if(points.size() == 0) 
	{
		printf("No hay deformadores para hacer el cálculo.\n");
		return -1;
	}

    // Init grid labels & weights
    for(unsigned int i = 0; i< grid.cells.size(); i++)
    {
        for(unsigned int j = 0; j< grid.cells[i].size(); j++)
        {
            for(unsigned int k = 0; k< grid.cells[i][j].size(); k++)
            {
                cell3d* cell = grid.cells[i][j][k];
                if(cell->getType() == EXTERIOR)
                {
                    // TODO: Estaría bien limpiarlo.
                }
                else
                {
                    if(onlyBorders && cell->getType() != BOUNDARY)
                    {
                        //cell->data->clear();
                        //cell->data->ownerLabel = -1;
                    }
                    else
                    {
                        //float distance = distancesFromEbeddedPoints(embeddedPoints[0],cell->data->embedding);
                        float distance = distancesFromEbeddedPointsExtended(embeddedPoints[0],cell->data->embedding, ((DefNode)points[0]).expansion);
                        int label = points[0].nodeId;

						int secondLabel = -1;
						float secondDistance = distance*99999;

						int lid = 0;
						int sid = -1;

                        cell->data->clear();
                        for(unsigned int id = 1; id< embeddedPoints.size(); id++)
                        {
                            //float newDist =  distancesFromEbeddedPoints(embeddedPoints[nodeId],cell->data->embedding);
                            float newDist =  distancesFromEbeddedPointsExtended(embeddedPoints[id],cell->data->embedding, points[id].expansion);
                            if(newDist < distance)
                            {
								// The first goes to second 
								secondDistance = distance;
								secondLabel = label;

								// We save the new first position
								distance = newDist;
								label = points[id].nodeId;

								sid = lid;
								lid = id;
							}
							else
							{
								// We test if is worst than the first but better than the second
								if(newDist < secondDistance)
								{
									secondDistance = newDist;
									secondLabel = points[id].nodeId;

									sid = id;
								}
							}
                        }

                        cell->data->segmentId = label;
                        cell->data->distanceToPos = distance;

						if(distance > 0)
						{
							float proportionalDistance = distancesFromEbeddedPointsExtended(embeddedPoints[lid],embeddedPoints[sid], 1.0);
							cell->data->confidenceLevel = ((secondDistance-distance)/proportionalDistance)/*/distance*/;
						}
						else
							cell->data->confidenceLevel = 10;
                    }
                }
            }
        }
    }

    // valueRange references the label count.
    grid.valueRange = points.size();
    return 0;
}


// Segment volume with labels constructing a voronoi diagram
int segmentVolumeWithEmbeddingPropagation( grid3d& grid,
                                vector<Point3d>& points,
                                vector< vector<double> >& embeddedPoints)
{

    // Init grid labels & weights
    for(unsigned int i = 0; i< grid.cells.size(); i++)
    {
        for(unsigned int j = 0; j< grid.cells[i].size(); j++)
        {
            for(unsigned int k = 0; k< grid.cells[i][j].size(); k++)
            {
                cell3d* cell = grid.cells[i][j][k];
                if(cell->getType() == EXTERIOR)
                {
                    // TODO: Estaría bien limpiarlo.
                }
                else cell->data->clear();
            }
        }
    }

    // TODEBUG: se puede optimizar lo de los frentes con un int en lugar de 3, pero hay que hacer una operación más
    // así ocuparía menos memoria y iría más ràpido en la construcción. También se puede optimizar construyendo vectores estáticos
    // a partir de una estimación del tamaño, ocupa más memoria, pero no perdemos el tiempo de construcción dinámica.

    vector< vector< Point3i > >* frontSelector[2];

    vector< vector< Point3i > > primaryFront;
    primaryFront.resize(points.size());

    vector< vector< Point3i > > secondaryFront;
    secondaryFront.resize(points.size());

    frontSelector[0] = &primaryFront;
    frontSelector[1] = &secondaryFront;

    int currentFront = 0;

    for(unsigned int ptId = 0; ptId < points.size(); ptId++)
    {
        // Set points label
        Point3i cell = grid.cellId(points[ptId]);
        (*frontSelector[currentFront])[ptId].push_back(cell);
    }

    printf("front propagation "); fflush(0);

    int iteration = 0;
    bool propagated= true;
    while(propagated)
    {
        propagated = frontPropagationWithEmbedding( grid, (*frontSelector[currentFront]), (*frontSelector[(currentFront+1)%2]), embeddedPoints, iteration);

        if(propagated) // Preparamos los datos para una vuelta más.
            currentFront = (currentFront+1)%2; // seleccionamos el otro frente

        iteration++;
    }

    printf("%d iterations\n", iteration); fflush(0);

    // valueRange references the label count.
    grid.valueRange = points.size();
    return 0;
}

// Init the the cells for optimized propagation
void initGridFlagsForFrontPropagationForSegmentation(grid3d& grid)
{
    for(unsigned int i = 1; i < grid.cells.size()-1; i++)
    {
        for(unsigned int j = 1; j < grid.cells[i].size()-1; j++)
        {
            for(unsigned int k = 1; k < grid.cells[i][j].size()-1; k++)
            {
                cell3d* cell = grid.cells[i][j][k];
                if(cell->getType() == BOUNDARY || cell->getType() == INTERIOR )
                {
                    cell->changed = false;
                    cell->data->itPass = 0;
                    cell->data->distanceToPos = MAX_LENGTH;
                    cell->data->pos = Point3i(-1,-1,-1);
                    cell->data->ownerLabel = -1;
                }
            }
        }
    }
}


bool frontPropagation(grid3d& grid,
                      vector<Point3i> & primaryFront,
                      vector<Point3i> & secondaryFront,
                      vector< DefNode > & points,
                      vector< vector< vector<float> > >& displacementMatrix,
                      int idFront,
                      int iter)
{
    bool propagation = false;
    // Donde guadaremos las celdas tratadas

    for(unsigned int frIdx = 0; frIdx < primaryFront.size(); frIdx++)
    {
        // Cogemos el último elemento del frente (ultimo por eficiencia)
        Point3i pt = primaryFront[frIdx];
        cell3d* cell = grid.cells[pt.X()][pt.Y()][pt.Z()];

        // Para cada elemento del frente realizamos la expansión.
        for(int up = -1; up <= 1; up++)
        {
            for(int side = -1; side <= 1; side++)
            {
                for(int frontw = -1; frontw <= 1; frontw++)
                {
                    // Nos saltamos la comparación con la misma celda
                    if(up == 0 && side == 0 && frontw == 0) continue;

                    cell3d* neighborCell = grid.cells[pt.X()+up][pt.Y()+side][pt.Z()+frontw];

                    // Comprobamos que esté en el borde o en el interior
                    if(neighborCell->getType() != BOUNDARY &&
                       neighborCell->getType() != INTERIOR)
                        continue;

                    // Si es del mismo grupo pasamos, es para cortar la expansión hacia dentro.
                    if(neighborCell->data->ownerLabel == cell->data->ownerLabel) continue;

                    // Añadimos el incremento de llegar a esa celda desde la actual.
                    double newDist = cell->data->distanceToPos + displacementMatrix[up+1][side+1][frontw+1];
                    //float newDist = Distance(points[idFront], grid.cellCenter(pt.X()+up,pt.Y()+side,pt.Z()+frontw));


                    if(neighborCell->data->distanceToPos > newDist)
                    {
                        neighborCell->data->distanceToPos = newDist;
                        neighborCell->data->ownerLabel = idFront;

                        Point3i aux(pt.X()+up,pt.Y()+side,pt.Z()+frontw);

                        //if(iter >= neighborCell->data->itPass)
                        //{
                            secondaryFront.push_back(aux);
                            neighborCell->data->itPass = iter;
                        //}

                        propagation = true;
                    }
                }
            }
        }
    }

    primaryFront.clear();

    return propagation;
}


// Segment volume with labels constructing a voronoi diagram
int segmentVolume( grid3d& grid,
                   vector< DefNode >& points)
{

    // Init grid labels
    /*
    for(unsigned int i = 0; i< grid.cells.size(); i++)
    {
        for(unsigned int j = 0; j< grid.cells[i].size(); j++)
        {
            for(unsigned int k = 0; k< grid.cells[i][j].size(); k++)
            {
                if(grid.cells[i][j][k]->getType() == EXTERIOR)
                {
                    // TODO: Estaría bien limpiarlo.
                }
                else
                {
                    grid.cells[i][j][k]->data->itPass = 0;
                }
            }
        }
    }
    */

    // TODEBUG: se puede optimizar lo de los frentes con un int en lugar de 3, pero hay que hacer una operación más
    // así ocuparía menos memoria y iría más ràpido en la construcción. También se puede optimizar construyendo vectores estáticos
    // a partir de una estimación del tamaño, ocupa más memoria, pero no perdemos el tiempo de construcción dinámica.

    vector< vector< Point3i > >* frontSelector[2];

    vector< vector< Point3i > > primaryFront;
    primaryFront.resize(points.size());

    vector< vector< Point3i > > secondaryFront;
    secondaryFront.resize(points.size());

    frontSelector[0] = &primaryFront;
    frontSelector[1] = &secondaryFront;

    int currentFront = 0;

    // Precomputacion de distancias entre celdas para calculo más eficiente
    vector< vector< vector<float> > > displacementMatrix;
    precomputeInterCellDistances( displacementMatrix, grid.cellSize);

    // Incializamos el grid
    initGridFlagsForFrontPropagationForSegmentation(grid);

    // Inicializamos los frentes
    for(unsigned int nodeId = 0; nodeId < points.size(); nodeId++)
    {
        // Set points label
        Point3i cell = grid.cellId(points[nodeId].pos);
        (*frontSelector[currentFront])[nodeId].push_back(cell);
        cell3d* pCell = grid.cells[cell.X()][cell.Y()][cell.Z()];
        pCell->data->ownerLabel = nodeId;
        pCell->data->distanceToPos = 0;
    }

    printf("front propagation "); fflush(0);
    int iteration = 0;
    bool propagated= true;
    while(propagated)
    {
        propagated = false;

        for(unsigned int idFront = 0; idFront< (*frontSelector[currentFront]).size(); idFront++)
        {
            propagated |= frontPropagation( grid, (*frontSelector[currentFront])[idFront], (*frontSelector[(currentFront+1)%2])[idFront], points, displacementMatrix, idFront, iteration);
        }

        currentFront = (currentFront+1)%2; // seleccionamos el otro frente

        printf("."); fflush(0);

        iteration++;
    }


    printf("%d iterations\n", iteration); fflush(0);

    // valueRange references the label count.
    grid.valueRange = points.size();
    return 0;
}


////////////////////////////////////////////////////
///////////////  BUILD VORONOI GRAPH   /////////////
////////////////////////////////////////////////////

void buildVoronoiGraph(grid3d& grid,
                       vector< vector<bool> >& voronoiGraph,
                       int graphSize,
                       bool onlyBorders)
{
    // Graph init
    voronoiGraph.resize(graphSize);
    for(int i = 0; i< graphSize; i++)
        voronoiGraph[i].resize(graphSize);

    for(unsigned int i = 0; i< voronoiGraph.size(); i++)
    {
        for(unsigned int j = 0; j< voronoiGraph[i].size(); j++)
        {
            voronoiGraph[i][j] = false;
        }
    }

    vector<Point3i> disp;
    disp.resize(6);
    disp[0] = Point3i(-1,0,0);
    disp[1] = Point3i(+1,0,0);
    disp[2] = Point3i(0,+1,0);
    disp[3] = Point3i(0,-1,0);
    disp[4] = Point3i(0,0,+1);
    disp[5] = Point3i(0,0,-1);

    for(unsigned int i = 1; i< grid.cells.size()-1; i=i+1)
    {
        for(unsigned int j = 1; j< grid.cells[i].size()-1; j=j+1)
        {
            for(unsigned int k = 1; k< grid.cells[i][j].size()-1; k=k+1)
            {
                cell3d* center = grid.cells[i][j][k];

                if(!onlyBorders)
                {
                    if(center->getType() == EXTERIOR || center->getType() == BOUNDARY ) continue;
                }
                else if(center->getType() != BOUNDARY ) continue;

                for(unsigned int d = 0; d < disp.size(); d++)
                {
                    cell3d* beside = grid.cells[i+disp[d].X()][j+disp[d].Y()][k+disp[d].Z()];

                    if(!onlyBorders)
                    {
                        if(beside->getType() == EXTERIOR || beside->getType() == BOUNDARY ) continue;
                    }
                    else if(beside->getType() != BOUNDARY ) continue;

                    if(center->data->ownerLabel != beside->data->ownerLabel)
                    {
                        assert(center->data->ownerLabel >= 0 && center->data->ownerLabel <graphSize);
                        assert(beside->data->ownerLabel >= 0 && beside->data->ownerLabel <graphSize);

                        voronoiGraph[center->data->ownerLabel][beside->data->ownerLabel] = true;
                        voronoiGraph[beside->data->ownerLabel][center->data->ownerLabel] = true;
                    }
                }
            }
        }
    }
}


////////////////////////////////////////////////////
///////////////  WEIGHTS COMPUTATION   /////////////
////////////////////////////////////////////////////

float computeWeightEmbedded(vector<double>& cellCtr,
                            vector<double>& nodePt,
                            float baseDistance,
                            bool invert,
                            double K,
                            double alpha)
{
    double S = 1;
    //double K = 100;
    //double alpha = 10;

    double partDistance = distancesFromEbeddedPoints(cellCtr, nodePt);

    double t = (partDistance/baseDistance);
    if(invert)
        t = 1-t;

    // Formula de smoothing
    float weight = S/(1.0+K*exp(-1.0*alpha*t));

    return weight;
}

float computeWeightProportional(float partDistance,
                                float baseDistance,
                                bool invert,
                                double K = 100,
                                double alpha = 10)
{
    double S = 1;

	// obtenemos lo que buscamos a partir de la distancia base.
    double t = (partDistance/baseDistance);
    if(invert)
        t = 1-t;

	float minV = S/(1.0+K*exp(-1.0*alpha*0));
	float maxV = S/(1.0+K*exp(-1.0*alpha*1.0));

    // Formula de kernel
    float weight = S/(1.0+K*exp(-1.0*alpha*t));

	// reescalado porque no se ajusta ni a 0, ni a 1.
	weight = (weight-minV)/(maxV-minV);

    return weight;
}

float computeWeight(Point3d cellCtr,
                    Point3d nodePt,
                    float baseDistance,
                    bool invert = true,
                    double K = 100,
                    double alpha = 10)
{
    return computeWeightProportional(interiorDistance(cellCtr, nodePt),baseDistance, invert, K, alpha);
}

void precomputeGraphDistances(vector< vector<double> >& points,
                              vector< vector<bool> >& voronoiGraph,
                              vector<int>& conexions,
                              vector<float>& meanDistances,
                              vector< vector<float> >& intraNodeDistance)
{
    double dist = 0;
    meanDistances.resize(voronoiGraph.size(), 0);
    intraNodeDistance.resize(voronoiGraph.size());

    for(unsigned int la001 = 0; la001 < voronoiGraph.size(); la001++)
    {
        intraNodeDistance[la001].resize(voronoiGraph[la001].size(), 0);
        int meanCount= 0;

        dist = 0;
        for(unsigned int lab = 0; lab< voronoiGraph[la001].size(); lab++)
        {
            if(voronoiGraph[la001][lab])
            {
                intraNodeDistance[la001][lab] = distancesFromEbeddedPoints(points[la001], points[lab]);
                dist += intraNodeDistance[la001][lab];
                meanCount++;
            }
        }

        if(meanCount > 0)
            meanDistances[la001] = dist/meanCount;
    }

    conexions.resize(voronoiGraph.size(), 0);
    for(unsigned int i = 0; i< voronoiGraph.size(); i++)
    {
        if(voronoiGraph[i][i])
        {
            printf("\nEl mismo nodo actua sobre si mismo... vaya\n"); fflush(0);
        }

        for(unsigned int j = 0; j< voronoiGraph[i].size(); j++)
        {
            if(voronoiGraph[i][j])
                conexions[i]++;
        }
    }
}

// Normaliza los pesos calculados
void normalizeWeights( grid3d& grid)
{
	// Esta deshabilitado para ver lo que se usa.
	assert(false);


	/*
    for(unsigned int i = 1; i < grid.cells.size()-1; i++)
    {
        for(unsigned int j = 1; j < grid.cells[i].size()-1; j++)
        {
            for(unsigned int k = 1; k < grid.cells[i][j].size()-1; k++)
            {
                cell3d* cell = grid.cells[i][j][k];
                float sumaWeights = 0;
                for(unsigned int w =0; w < cell->data->weights.size(); w++)
                {
                    sumaWeights += cell->data->weights[w];
                }

                if(sumaWeights > 0)
                {
                    for(unsigned int ww =0; ww< cell->data->weights.size(); ww++)
                    {
                        cell->data->weights[ww] = cell->data->weights[ww]/sumaWeights;
                    }
                }
            }
        }
    }
	*/

}

////////////////////////////////////////////////////
///////////////  WEIGHTS PROPAGATION   /////////////
////////////////////////////////////////////////////

// Init the the cells for optimized propagation
void initGridFlagsForFrontPropagation(grid3d& grid, int idFront)
{
    for(unsigned int i = 1; i < grid.cells.size()-1; i++)
    {
        for(unsigned int j = 1; j < grid.cells[i].size()-1; j++)
        {
            for(unsigned int k = 1; k < grid.cells[i][j].size()-1; k++)
            {
                cell3d* cell = grid.cells[i][j][k];
				
				if(cell->getType() == BOUNDARY)
				{
					int owner = cell->data->ownerLabel;
                    cell->changed = false;
                    cell->data->itPass = 0;
                    if(owner == idFront)
                    {
                        cell->data->distanceToPos = 0;
                        cell->data->pos = Point3i(i,j,k);
                        cell->data->auxInfluences.push_back(weight(idFront, 1.0));
                    }
                    else
                    {
                        cell->data->distanceToPos = MAX_LENGTH;
                        cell->data->pos = Point3i(-1,-1,-1);
                        cell->data->ownerWeight = 0;
                    }
                }
            }
        }
    }
}

// Propaga el peso a lo largo de la superficie, teniendo en cuenta una segmentación.
void borderFrontSmoothing(grid3d& grid,
                          vector< Point3i >& front,
                          vector<Point3i>& postProcess,
                          float smoothPropagationRatio,
                          int idFront)
{
    vector< Point3i >* frontSelector[2];

    vector< Point3i > secondaryFront;
    secondaryFront.clear();

    frontSelector[0] = &front;
    frontSelector[1] = &secondaryFront;

    int currentFront = 0;
    int iter = 0;

    float baseDistance = smoothPropagationRatio;

    // Precomputacion de distancias entre celdas para calculo más eficiente
    vector< vector< vector<float> > > displacementMatrix;
    precomputeInterCellDistances( displacementMatrix, grid.cellSize);

    bool propagation = true;
    while(propagation)
    {
        propagation = false;
        // Donde guadaremos las celdas tratadas
        int nextFront = (currentFront+1)%2;

        for(unsigned int frIdx = 0; frIdx < (*frontSelector[currentFront]).size(); frIdx++)
        {
            // Cogemos el último elemento del frente (ultimo por eficiencia)
            Point3i pt = (*frontSelector[currentFront])[frIdx];
            cell3d* cell = grid.cells[pt.X()][pt.Y()][pt.Z()];

            // Para cada elemento del frente realizamos la expansión.
            for(int up = -1; up <= 1; up++)
            {
                for(int side = -1; side <= 1; side++)
                {
                    for(int frontw = -1; frontw <= 1; frontw++)
                    {
                        // Nos saltamos la comparación con la misma celda
                        if(up == 0 && side == 0 && frontw == 0) continue;

                        cell3d* neighborCell = grid.cells[pt.X()+up][pt.Y()+side][pt.Z()+frontw];

                        // Comprobamos que esté en el borde.
                        if(neighborCell->getType() != BOUNDARY) continue;

                        // Si es del mismo grupo pasamos, es para cortar la expansión hacia dentro.
						if(neighborCell->data->ownerLabel == idFront) continue;

                        // Añadimos el incremento de llegar a esa celda desde la actual.
                        float newDist = cell->data->distanceToPos + displacementMatrix[up+1][side+1][frontw+1];

                        // Si llegar es más demasiado costoso cortamos.
                        if(newDist > baseDistance) continue;

                        if(neighborCell->data->distanceToPos > newDist)
                        {
                            neighborCell->data->distanceToPos = newDist;
                            //neighborCell->data->pos = origPt;

                            Point3i aux(pt.X()+up,pt.Y()+side,pt.Z()+frontw);
                            if(iter >= neighborCell->data->itPass)
                            {
                                (*frontSelector[nextFront]).push_back(aux);
                                neighborCell->data->itPass = iter;
                                // Para postprocessar
                                postProcess.push_back(aux);
                            }

                            propagation = true;
                        }
                    }
                }
            }
        }

        (*frontSelector[currentFront]).clear();
        iter++;
        currentFront = nextFront; // seleccionamos el otro frente
    }

    // DEBUG COMMENTS
    //printf("Propagacion con valores k=%f y alpha=%f\n", grid.kValue, grid.alphaValue);

    for(unsigned int i = 0; i<postProcess.size(); i++)
    {
        Point3i pt = postProcess[i];
        cell3d* cell = grid.cells[pt.X()][pt.Y()][pt.Z()];

        if(cell->changed) continue;

        //if(cell->data->ownerLabel == idFront) continue;

        // calculamos la influencia según la distancia propagada por la superficie.
        float weightValue = computeWeightProportional(cell->data->distanceToPos, baseDistance,
                                                      true, grid.kValue, grid.alphaValue);

		cell->data->auxInfluences.push_back(weight(idFront, weightValue));

        // Marcamos que ya hemos hecho el cálculo para esta celda.
        cell->changed = true;
    }
}

// Propaga el peso a lo largo de la superficie, teniendo en cuenta una segmentación.
void borderFrontPropagation(grid3d& grid,
                            vector< Point3i >& front,
                            vector< vector<bool> >& voronoiGraph,
                            vector<float>& intraNodeDistance,
                            int idFront)
{
	assert(false); // disabled for flux control verification

	/*
    vector<Point3i> postProcess;

    vector< Point3i >* frontSelector[2];

    vector< Point3i > secondaryFront;
    secondaryFront.clear();

    frontSelector[0] = &front;
    frontSelector[1] = &secondaryFront;

    int currentFront = 0;
    int iter = 0;

    float baseDistance = 0;

    int count = 0;
    for(unsigned int i = 0; i<intraNodeDistance.size(); i++)
    {
        if(voronoiGraph[idFront][i])
        {
            baseDistance += intraNodeDistance[i];
            count++;
        }
    }

    if(count > 0) baseDistance = (baseDistance/count);

    //assert(baseDistance != 0);

    if(baseDistance == 0)
    {
        printf("OJO!! Hay un punto que es igual a otro... esto hay que gestionarlo.\n"); fflush(0);
        front.clear();
        return;
    }

    // Precomputacion de distancias entre celdas para calculo más eficiente
    vector< vector< vector<float> > > displacementMatrix;
    precomputeInterCellDistances( displacementMatrix, grid.cellSize);

    bool propagation = true;
    while(propagation)
    {
        propagation = false;
        // Donde guadaremos las celdas tratadas
        int nextFront = (currentFront+1)%2;

        for(unsigned int frIdx = 0; frIdx < (*frontSelector[currentFront]).size(); frIdx++)
        {
            // Cogemos el último elemento del frente (ultimo por eficiencia)
            Point3i pt = (*frontSelector[currentFront])[frIdx];
            cell3d* cell = grid.cells[pt.X()][pt.Y()][pt.Z()];

            //Point3i origPt = cell->data->pos;
            //cell3d* frontCell = grid.cells[origPt.X()][origPt.Y()][origPt.Z()];

            // Para cada elemento del frente realizamos la expansión.
            for(int up = -1; up <= 1; up++)
            {
                for(int side = -1; side <= 1; side++)
                {
                    for(int frontw = -1; frontw <= 1; frontw++)
                    {
                        // Nos saltamos la comparación con la misma celda
                        if(up == 0 && side == 0 && frontw == 0) continue;

                        cell3d* neighborCell = grid.cells[pt.X()+up][pt.Y()+side][pt.Z()+frontw];

                        // Comprobamos que esté en el borde.
                        if(neighborCell->getType() != BOUNDARY) continue;

                        // Si es del mismo grupo pasamos, es para cortar la expansión hacia dentro.
                        if(neighborCell->data->ownerLabel == idFront) continue;

                        //float newDist = distancesFromEbeddedPoints(frontCell->data->embedding, neighborCell->data->embedding);

                        // Añadimos el incremento de llegar a esa celda desde la actual.
                        //float newDist = cell->data->distanceToPos + (grid.cellCenter(pt.X(),pt.Y(),pt.Z()) - grid.cellCenter(pt.X()+up,pt.Y()+side,pt.Z()+frontw)).Norm();
                        float newDist = cell->data->distanceToPos + displacementMatrix[up+1][side+1][frontw+1];

                        // Si llegar es más demasiado costoso cortamos.
                        if(newDist > baseDistance) continue;

                        if(neighborCell->data->distanceToPos > newDist)
                        {
                            neighborCell->data->distanceToPos = newDist;
                            //neighborCell->data->pos = origPt;

                            Point3i aux(pt.X()+up,pt.Y()+side,pt.Z()+frontw);
                            if(iter >= neighborCell->data->itPass)
                            {
                                (*frontSelector[nextFront]).push_back(aux);
                                neighborCell->data->itPass = iter;
                                // Para postprocessar
                                postProcess.push_back(aux);
                            }

                            propagation = true;
                        }
                    }
                }
            }
        }

        (*frontSelector[currentFront]).clear();

        iter++;
        // el frente tiene que quedar vaciO

        currentFront = nextFront; // seleccionamos el otro frente
    }

    for(unsigned int i = 0; i<postProcess.size(); i++)
    {
        Point3i pt = postProcess[i];
        cell3d* cell = grid.cells[pt.X()][pt.Y()][pt.Z()];

        if(cell->changed) continue;

        if(cell->data->ownerLabel == idFront) continue;

        //cell3d* cellOrig = grid.cells[cell->data->pos.X()][cell->data->pos.Y()][cell->data->pos.Z()];

        // calculamos la influencia según la distancia propagada por la superficie.
        float weightValue = computeWeightProportional(cell->data->distanceToPos, baseDistance, true, grid.kValue, grid.alphaValue);
        //float weightValue = computeWeightEmbedded(cell->data->embedding, cellOrig->data->embedding, baseDistance, true, grid.kValue, grid.alphaValue);

        cell->data->labels.push_back(idFront);
        cell->data->weights.push_back(weightValue);

        // Marcamos que ya hemos hecho el cálculo para esta celda.
        cell->changed = true;
    }
	*/
}

// Interior weights propagation using embedded distances
int weightsComputationEmbedded(grid3d& grid,
                               vector< vector<double> >& points,
                               vector< vector<bool> >& voronoiGraph)
{
	/*
    vector<int> conexions;
    vector<float> meanDistances;
    vector< vector<float> > intraNodeDistance;

    precomputeGraphDistances(points,voronoiGraph, conexions, meanDistances,intraNodeDistance);

    // Recorremos las celdas y asignamos un peso para cada punto
    for(unsigned int i = 0; i< grid.cells.size(); i++)
    {
        for(unsigned int j = 0; j< grid.cells[i].size(); j++)
        {
            for(unsigned int k = 0; k< grid.cells[i][j].size(); k++)
            {

                if(grid.cells[i][j][k]->getType() == EXTERIOR)
                    continue;

                else
                {
                    double totalWeight = 0;

                    unsigned int label = grid.cells[i][j][k]->data->ownerLabel;

                    // Inicializamos el espacio para las influencias, ponemos +1 para la del propio nodo.
                    grid.cells[i][j][k]->data->labels.resize(conexions[label]+1);
                    grid.cells[i][j][k]->data->weights.resize(conexions[label]+1);

                    vector<double> euclideanCell(3);

                    // Usaremos el punto del centro como si fuera un embedding de 3Dimensiones
                    if(!grid.metricUsed)
                    {
                        Point3d temp = grid.cellCenter(i,j,k);
                        euclideanCell[0] = temp.X();
                        euclideanCell[1] = temp.Y();
                        euclideanCell[2] = temp.Z();
                    }

                    // calculamos el peso con su mismo nodo

                    double baseDistance = 0;

                    baseDistance = meanDistances[label];

                    // Calculamos el peso que ejerce el mismo nodo
                    float weight = 0;
                    if(grid.metricUsed)
                       weight = computeWeightEmbedded(grid.cells[i][j][k]->data->embedding, points[label],
                                                      baseDistance, true, grid.kValue, grid.alphaValue);
                    else
                    {
                        weight = computeWeightEmbedded(euclideanCell, points[label],
                                                       baseDistance, true, grid.kValue, grid.alphaValue);
                    }

                    grid.cells[i][j][k]->data->labels[0] = label;
                    grid.cells[i][j][k]->data->weights[0] = weight;
                    totalWeight += weight;

                    // calculamos el peso con los nodos vecinos.
                    int count = 1;
                    for(unsigned int lab = 0; lab< voronoiGraph.size(); lab++)
                    {
                        if(lab == label) continue;

                        if(voronoiGraph[label][lab])
                        {
                            //baseDistance = distancesFromEbeddedPoints(points[label], points[lab]);
                            baseDistance = meanDistances[lab];
                            float weight = 0;

                            if(grid.metricUsed)
                                weight = computeWeightEmbedded(grid.cells[i][j][k]->data->embedding, points[lab], baseDistance,
                                                               true, grid.kValue, grid.alphaValue);
                            else
                                weight = computeWeightEmbedded(euclideanCell, points[lab], baseDistance,
                                                               true, grid.kValue, grid.alphaValue);

                            grid.cells[i][j][k]->data->labels[count] = lab;
                            grid.cells[i][j][k]->data->weights[count] = weight;
                            totalWeight += weight;
                            count++;
                        }
                    }

                    // Normalizacion
                    if(totalWeight > 0)
                    for(unsigned int wi = 0; wi < grid.cells[i][j][k]->data->weights.size(); wi++)
                       grid.cells[i][j][k]->data->weights[wi] = grid.cells[i][j][k]->data->weights[wi]/totalWeight;
                }

            }
        }
    }
	*/
    return 0;
}


// Inteiror weights propagation using euclidean distances
int weightsComputation(grid3d& grid,
                       vector<Point3d>& points,
                       vector< vector<bool> >& voronoiGraph)
{
	/*
    vector<int> conexions;
    conexions.resize(voronoiGraph.size(), 0);
    for(unsigned int i = 0; i< voronoiGraph.size(); i++)
    {
        for(unsigned int j = 0; j< voronoiGraph[i].size(); j++)
        {
            conexions[i]++;
        }
    }

    // Recorremos las celdas y asignamos un peso para cada punto
    for(unsigned int i = 0; i< grid.cells.size(); i++)
    {
        for(unsigned int j = 0; j< grid.cells[i].size(); j++)
        {
            for(unsigned int k = 0; k< grid.cells[i][j].size(); k++)
            {
                if(grid.cells[i][j][k]->getType() == EXTERIOR)
                    continue;

                else
                {
                    int label = grid.cells[i][j][k]->data->ownerLabel;

                    // Inicializamos el espacio para las influencias, ponemos +1 para la del propio nodo.
                    grid.cells[i][j][k]->data->labels.resize(conexions[label]+1);
                    grid.cells[i][j][k]->data->weights.resize(conexions[label]+1);

                    // calculamos el peso con su mismo nodo
                    double baseDistance = 0;
                    int meanCount = 0;
                    for(unsigned int lab = 0; lab< voronoiGraph[label].size(); lab++)
                    {
                        if(voronoiGraph[label][lab])
                        {
                            baseDistance += interiorDistance(points[label], points[lab]);
                            meanCount++;
                        }
                    }
                    baseDistance = baseDistance/meanCount;
                    float weight = computeWeight(grid.cellCenter(i,j,k), points[label], baseDistance);
                    grid.cells[i][j][k]->data->labels[0] = label;
                    grid.cells[i][j][k]->data->weights[0] = weight;

                    // calculamos el peso con los nodos vecinos.
                    int count = 1;
                    for(unsigned int lab = 0; lab< voronoiGraph.size(); lab++)
                    {
                        if(voronoiGraph[label][lab])
                        {
                            baseDistance = interiorDistance(points[label], points[lab]);
                            float weight = computeWeight(grid.cellCenter(i,j,k), points[lab], baseDistance);
                            grid.cells[i][j][k]->data->labels[count] = lab;
                            grid.cells[i][j][k]->data->weights[count] = weight;
                        }
                    }
                }
            }
        }
    }

	*/
    return 0;
}

// Crea un frent de avance según el nodeId especificado.
// Si no hay ninguna celda con el ese nodoId, devuelve un frente vacío.
void initfrontWithBoundaries(grid3d& grid, vector< Point3i >& front, int nodeId)
{
    for(unsigned int i = 1; i < grid.cells.size()-1; i++)
    {
        for(unsigned int j = 1; j < grid.cells[i].size()-1; j++)
        {
            for(unsigned int k = 1; k < grid.cells[i][j].size()-1; k++)
            {
				cell3d* cell = grid.cells[i][j][k];
				if(cell->getType() == BOUNDARY)
				{
					cell->data->itPass = 0;
					cell->data->ownerWeight = 0;

					if(cell->data->ownerLabel != nodeId) continue;
	           
					bool border = false;
                
                    for(int up = -1; up <= 1 ; up++)
                    {
                        for(int side = -1; side <= 1 ; side++)
                        {
                            for(int frontw = -1; frontw <= 1 ; frontw++)
                            {
                                if(up == 0 && side == 0 && frontw == 0) continue;

                                cell3d* neighborCell = grid.cells[i+up][j+side][k+frontw];
                                
								if(neighborCell->getType() != BOUNDARY) continue;

                                if(neighborCell->data->ownerLabel != cell->data->ownerLabel)
                                {
                                    if(!border) 
										front.push_back(Point3i(i,j,k));

                                    cell->frontier = true;
                                    border = true;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}


// inicializa el grid y los frentes de avance para hacer bien la expansión.
// Detecta los bordes entre regiones.

void initfrontsWithBoundaries(grid3d& grid, vector< vector< Point3i > >& fronts)
{
	assert(false); // Disabled for flux verification.

	/*
    for(unsigned int i = 1; i < grid.cells.size()-1; i++)
    {
        for(unsigned int j = 1; j < grid.cells[i].size()-1; j++)
        {
            for(unsigned int k = 1; k < grid.cells[i][j].size()-1; k++)
            {
                cell3d* cell = grid.cells[i][j][k];
                cell->data->weights.clear();
                cell->data->labels.clear();
                cell->data->itPass = 0;

                bool border = false;
                if(cell->getType() == BOUNDARY)
                {
                    for(int up = -1; up <= 1 ; up++)
                    {
                        for(int side = -1; side <= 1 ; side++)
                        {
                            for(int frontw = -1; frontw <= 1 ; frontw++)
                            {
                                if(up == 0 && side == 0 && frontw == 0) continue;

                                cell3d* neighborCell = grid.cells[i+up][j+side][k+frontw];

                                if(neighborCell->getType() != BOUNDARY) continue;

                                if(neighborCell->data->ownerLabel != cell->data->ownerLabel)
                                {
                                    if(!border)
                                        fronts[cell->data->ownerLabel].push_back(Point3i(i,j,k));

                                    cell->frontier = true;
                                    border = true;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
	*/
}




int PropagateWeights(grid3d& grid,
                     vector< DefNode >& points,
                     vector< vector<bool> >& voronoiGraph,
                     bool onlyBorders)
{
    // PRECONDITIONS
    // Grafo construido
    // Grid Segmentado -> quizás más adelante podemos fusionar estos pasos con el que viene a continuación

    // Fronts creation, as many as labels
    vector< vector< Point3i > > fronts;
    fronts.resize(points.size());

    // Insert in front depending on the boundaries.
    initfrontsWithBoundaries(grid, fronts);

    vector<int> conexions;
    vector<float> meanDistances;
    vector< vector<float> > intraNodeDistance;

    // Precalculamos algunos datos para ir más rápido.
    double dist = 0;
    meanDistances.resize(voronoiGraph.size(), 0);
    intraNodeDistance.resize(voronoiGraph.size());

    for(unsigned int la001 = 0; la001 < voronoiGraph.size(); la001++)
    {
        intraNodeDistance[la001].resize(voronoiGraph[la001].size(), 0);
        int meanCount= 0;

        dist = 0;
        for(unsigned int lab = 0; lab< voronoiGraph[la001].size(); lab++)
        {
            //if(voronoiGraph[la001][lab])
            {
                intraNodeDistance[la001][lab] = (points[la001].pos-points[lab].pos).Norm();//distancesFromEbeddedPoints(embeddedPoints[la001], embeddedPoints[lab]);
                //intraNodeDistance[lab][label] = intraNodeDistance[label][lab];
                dist += intraNodeDistance[la001][lab];
                meanCount++;
            }
        }

        if(meanCount > 0)
            meanDistances[la001] = dist/meanCount;
    }

    conexions.resize(voronoiGraph.size(), 0);
    for(unsigned int i = 0; i< voronoiGraph.size(); i++)
    {
        if(voronoiGraph[i][i])
        {
            printf("\nEl mismo nodo actua sobre si mismo... vaya\n"); fflush(0);
        }

        for(unsigned int j = 0; j< voronoiGraph[i].size(); j++)
        {
            if(voronoiGraph[i][j])
                conexions[i]++;
        }
    }


    for(unsigned int idFront = 0; idFront < fronts.size(); idFront++)
    {
        // Init grid flags for front propagations.
        initGridFlagsForFrontPropagation(grid, idFront);


        borderFrontPropagation(grid, fronts[idFront], voronoiGraph,
                                intraNodeDistance[idFront],idFront);


    }

    normalizeWeights(grid);

    return 0;
}


// PRECONDITIONS
// Grafo construido
// Grid Segmentado -> quizás más adelante podemos fusionar estos pasos con el que viene a continuación
int PropagateWeightsFromSkt(grid3d& grid,
                     skeleton* skt,
                     float smoothPropagationRatio,
                     bool onlyBorders)
{
    // Fronts creation, as many as labels
    vector< vector< Point3i > > fronts;
    fronts.resize(skt->getBoneCount());

    // Insert in front depending on the boundaries.
    initfrontsWithBoundaries(grid, fronts);

    for(unsigned int idFront = 0; idFront < fronts.size(); idFront++)
    {
        // Init grid flags for front propagations.
        initGridFlagsForFrontPropagation(grid,
                                         idFront);

        vector<Point3i> postProcess;

        borderFrontSmoothing(grid,
                             fronts[idFront],
                             postProcess,
                             smoothPropagationRatio,
                             idFront);

    }

    normalizeWeights(grid);

    return 0;
}

void ApplyWeightLimitedByHierarchy(grid3d& grid, vector<Point3i>& postProcess, int fatherNodeId, int nodeId)
{
	/*
    //for(unsigned int i = 0; i < postProcess.size(); i++)
    //{
    for(unsigned int i = 0; i < grid.cells.size(); i++)
    {
        for(unsigned int j = 0; j < grid.cells[i].size(); j++)
        {
            for(unsigned int k = 0; k < grid.cells[i][j].size(); k++)
            {
                //cell3d* cell = grid.cells[postProcess[i].X()][postProcess[i].Y()][postProcess[i].Z()];
                cell3d* cell = grid.cells[i][j][k];

                int idx = indexOf(cell->data->labels,fatherNodeId);
                float auxWeight = cell->data->auxWeight;
                if( idx >= 0)
                {
                    if(cell->data->weights[idx] <= auxWeight)
                    {
                        // El peso del padre se recorta para ceder al hijo.
                        cell->data->weights[idx] -= auxWeight;

                        // TODEBUG:
                        // podemos limpiar la influencia 0.

                        cell->data->labels.push_back(nodeId);
                        cell->data->weights.push_back(auxWeight);
                    }
                    else
                    {
                        // El peso del padre pasa al hijo por completo.
                        cell->data->labels.push_back(nodeId);
                        cell->data->weights.push_back(cell->data->weights[idx]);
                        cell->data->weights[idx] = 0;
                    }
                }
                else
                {
                    // Al no haber peso del padre que repartir: el peso queda en 0, por lo que no aplicamos nada.
                }
            }
        }
    }
	*/
}

int PropagateWeightsFromJoint(grid3d& grid,
                              int fatherNodeId,
                              int boneId,
                              float smoothPropagationRatio,
                              bool onlyBorders)
{
	/*
    // Fronts creation, as many as labels
    vector< Point3i > front;

    bool partialSegmentation = true;

    // Insert in front depending on the boundaries.
    initfrontWithBoundaries(grid, front, boneId);

    // Init grid flags for front propagations.
    initGridFlagsForFrontPropagation(grid, boneId);

    // Smoothing sin corte jerárquico.
    vector<Point3i> postProcess;
    borderFrontSmoothing(grid, front, postProcess, smoothPropagationRatio, boneId, partialSegmentation);

    // Apply filtered weights
    ApplyWeightLimitedByHierarchy(grid, postProcess, fatherNodeId, boneId);

    // TODEBUG: no sé si es bueno normalizar cada vez.
    //normalizeWeights(grid);

	*/
    return 0;
}

////////////////////////////////////////////////////
///////////////  SKINNING COMPUTATION   ////////////
////////////////////////////////////////////////////

void gridInit(MyMesh& mesh,
               grid3d& grid)
{
    // Agrandamos un 1% la caja contenedora
    Box3d newBound = mesh.bbox;
    double plusX = newBound.DimX()*0.1;
    double plusY = newBound.DimY()*0.1;
    double plusZ = newBound.DimZ()*0.1;

    newBound.min -= Point3d(plusX, plusY, plusZ);
    newBound.max += Point3d(plusX, plusY, plusZ);

    int resolution = (int)pow(2.0,grid.res);
    Point3i res(resolution, resolution, resolution);

    //Iniciamos el grid
    double cellSize = 0;
    if(newBound.DimX() < newBound.DimY())
    {
        if(newBound.DimX() < newBound.DimZ())
        {
            cellSize = newBound.DimX()/resolution;
            res.Y() = (int)ceil(newBound.DimY()/cellSize);
            res.Z() = (int)ceil(newBound.DimZ()/cellSize);

        }
        else
        {
            cellSize = newBound.DimZ()/resolution;
            res.Y() = (int)ceil(newBound.DimY()/cellSize);
            res.X() = (int)ceil(newBound.DimX()/cellSize);
        }
    }
    else
    {
        if(newBound.DimY() > newBound.DimZ())
        {
            cellSize = newBound.DimY()/resolution;
            res.X() = (int)ceil(newBound.DimX()/cellSize);
            res.Z() = (int)ceil(newBound.DimZ()/cellSize);
        }
        else
        {
            cellSize = newBound.DimZ()/resolution;
            res.Y() = (int)ceil(newBound.DimY()/cellSize);
            res.X() = (int)ceil(newBound.DimX()/cellSize);
        }
    }

    newBound.max = newBound.min + Point3d(res.X(), res.Y(), res.Z())*cellSize;
    grid.init(newBound, res, mesh.vn);
}

int gridCellsEmbeddingInterpolation(MyMesh& mesh,
                                    grid3d& grid,
                                    vector< vector<double> >& embedding,
                                    bool onlyBorders)
{
	int totalBorder = 0;
	for(unsigned int i = 0; i< grid.cells.size(); i=i+1)
	{
		for(unsigned int j = 0; j< grid.cells[i].size(); j=j+1)
		{
			for(unsigned int k = 0; k< grid.cells[i][j].size(); k=k+1)
			{
				cell3d* center = grid.cells[i][j][k];
				if(onlyBorders && center->getType() != BOUNDARY) continue;
				if(center->getType() == EXTERIOR ) continue;
				totalBorder++;
			}
		}
	}
	cout << "Interpolation of " << totalBorder;
	cout << "border cells:\n" ;
    int interiorCells = 0;
    for(unsigned int i = 0; i< grid.cells.size(); i=i+1)
    {
        for(unsigned int j = 0; j< grid.cells[i].size(); j=j+1)
        {
            for(unsigned int k = 0; k< grid.cells[i][j].size(); k=k+1)
            {
                cell3d* center = grid.cells[i][j][k];

                if(onlyBorders && center->getType() != BOUNDARY) continue;
                if(center->getType() == EXTERIOR ) continue;

                Point3d cellcenter = grid.cellCenter(i,j,k);
                mvcEmbedPoint(cellcenter, center->data->embedding, embedding, mesh);

                interiorCells++;
            }
        }
		cout << "Procesed: " << float(interiorCells)/float(totalBorder) << "%" << endl;
		grid.SafefPrintProcessStatus(int(float(interiorCells)/float(totalBorder)*100.0));
    }

    return interiorCells;
}

void computeSkinning(MyMesh& mesh,
                     grid3d& grid,
                     voronoiGraphData& v,
                     vector< vector<double> >& embeddedPoints,
                     vector< vector<double> >& embedding,
                     bool onlyBorders)
{
    clock_t begin, end;

    if(VERBOSE) begin=clock();

    if(VERBOSE)
    {
        cout << "------------------------------------------------------" << endl;
        cout << "                  - PREPROCESO -                      " << endl;
        cout << "------------------------------------------------------" << endl;
        cout << "Creación del grid: ";
		cout << grid.res << endl;
    }

    // CONSTRUCCION DEL GRID
    gridInit(mesh,grid);

    if(VERBOSE)
    {
        end=clock();
        cout << double(timelapse(end,begin)) << " s"<< endl;
        begin = end;
        cout << "Etiquetado del grid: ";
    }

    //Tipificamos las celdas según si es interior, exterior, o borde de la maya
    grid.typeCells(mesh);

    if(VERBOSE)
    {
        end=clock();
        cout << double(timelapse(end,begin)) << " s"<< endl;
        cout << "Grid cells embedding";
        begin=clock();
    }

    // EMBEDING INTERPOLATION FOR EVERY CELL
    int interiorCells = 0;
    interiorCells = gridCellsEmbeddingInterpolation(mesh, grid, embedding, onlyBorders);

    if(VERBOSE)
    {
        end=clock();
        cout << "("<< interiorCells <<"cells): " << double(timelapse(end,begin)) << " s"<< endl;

        //if(grid.metricUsed)
        //{
            double memorySize = (double)(interiorCells*DOUBLESIZE*embedding[0].size())/MBSIZEINBYTES;
            cout << "Estimated memory consumming: " << memorySize << "MB" <<endl;
        //}
        cout << "------------------------------------------------------" << endl;
        cout << "                  - PROCESO ONLINE -                  " << endl;
        cout << "------------------------------------------------------" << endl;
        cout << "Segmentacion del volumen: " ;
        begin=clock();
    }

    // SEGMENTACION DEL VOLUMEN

    if(grid.metricUsed)
    {
        segmentVolumeWithEmbedding(grid, v.intPoints, embeddedPoints, onlyBorders);
    }
    else
    {
        segmentVolume(grid, v.intPoints);
    }

    if(VERBOSE)
    {
        end=clock();
        cout << double(timelapse(end,begin)) << " s"<< endl;
        cout << "Construccion del grafo de voronoi: " ;
        begin=clock();
    }

    // BUILD VORONOI GRAPH
    buildVoronoiGraph(grid, v.voronoiGraph, v.intPoints.size(), onlyBorders);

    if(VERBOSE)
    {
        end=clock();
        cout << double(timelapse(end,begin)) << " s"<< endl;
        cout << "Propagación de pesos: " ;
        begin=clock();
    }

    // WEIGHTS PROPAGATION & COMPUTATION
    PropagateWeights(grid, v.intPoints, v.voronoiGraph, onlyBorders);

    if(VERBOSE)
    {
        end=clock();
        cout << double(timelapse(end,begin)) << " s"<< endl;
        fflush(0);
    }
}


					 // Marcamos la tabla de traducción de cada nodo
void createTraductionTable(joint* jt, map<int, int>& traductionTable, int idNode, bool onlyRoot)
{
    // Los nodos correspondientes al hueso jt
    for(unsigned int i = 0; i< jt->nodes.size(); i++)
    {
        traductionTable[((DefNode*)(jt->nodes[i]))->nodeId] = idNode;
    }

	// Si solo queremos anadir la base nos quedamos aqui.
	if(onlyRoot) return;

    // Descendemos por la jerarquía marcando los nodos de los huesos hijos.
    for(unsigned int jtChild = 0; jtChild < jt->childs.size(); jtChild++)
    {
        createTraductionTable(jt->childs[jtChild], traductionTable, idNode);
    }
}

void initDomainForId(grid3d& grid, int fatherId)
{
	for(unsigned int i = 0; i< grid.cells.size(); i++)
    {
        for(unsigned int j = 0; j< grid.cells[i].size(); j++)
        {
            for(unsigned int k = 0; k< grid.cells[i][j].size(); k++)
            {
				if(!(grid.cells[i][j][k]->data) || grid.cells[i][j][k]->getType() != BOUNDARY ) continue;

                cell3d* cell = grid.cells[i][j][k];
				
				if(fatherId < 0)
				{
					cell->data->domain = 1; // Primera inicialización.
					cell->data->domainId = -1;
				}
				else
				{
					cell->data->domain = cell->data->getDomain(fatherId); // Buscamos el peso del padre.
					cell->data->domainId = fatherId;
				}
			}
		}
	}
}

// Crea un frent de avance según el nodeId especificado.
// Si no hay ninguna celda con el ese nodoId, devuelve un frente vacío.
void initCellWeightsSmoothing(grid3d& grid, vector< Point3i >& front, int nodeId)
{
	front.clear();
    for(unsigned int i = 0; i < grid.cells.size(); i++)
    {
        for(unsigned int j = 0; j < grid.cells[i].size(); j++)
        {
            for(unsigned int k = 0; k < grid.cells[i][j].size(); k++)
            {
				cell3d* cell = grid.cells[i][j][k];
				if(cell->getType() == BOUNDARY)
				{
					if(cell->data->ownerLabel != nodeId)
					{
						cell->changed = false;
						cell->data->itPass = 0;
						cell->data->ownerWeight = 0.0;
						cell->data->tempOwnerWeight = 0.0;
					}
					else
					{
						cell->changed = true;
						cell->data->itPass = 0;
						cell->data->ownerWeight = 1.0;
						cell->data->tempOwnerWeight = 1.0;
						front.push_back(Point3i(i,j,k));
					}
                }
            }
        }
    }
}

// Propaga el peso a lo largo de la superficie, teniendo en cuenta una segmentación.
void weightsSmoothing(grid3d& grid,
                      vector< Point3i >& front,
                      float smoothPropagationRatio,
                      int idFront,
					  int smoothingPasses)
{
    int iter = 0;
	int distance = 4;

	double weightsSum;
	float weightsCount;
    while(iter < smoothingPasses)
    {
        for(unsigned int frIdx = 0; frIdx < front.size(); frIdx++)
        {
            Point3i pt = front[frIdx];
            cell3d* cell = grid.cells[pt.X()][pt.Y()][pt.Z()];

			weightsSum = 0;
			weightsCount = 0;
            // Para cada elemento del frente realizamos la expansión.
            for(int up = -distance; up <= distance; up++)
            {
                for(int side = -distance; side <= distance; side++)
                {
                    for(int frontw = -distance; frontw <= distance; frontw++)
                    {
                        // Nos saltamos la comparación con la misma celda
                        if(up == 0 && side == 0 && frontw == 0) continue;
						if(abs(up) == 1 && abs(side) == 1 && abs(frontw) == 1) continue; // extremos

                        cell3d* neighborCell = grid.cells[pt.X()+up][pt.Y()+side][pt.Z()+frontw];
						if(!neighborCell) continue;

                        // Comprobamos que esté en el borde.
                        if(neighborCell->getType() != BOUNDARY) continue;
						
						// Si no esta encolada... la ponemos
						if(!neighborCell->changed)
						{
							neighborCell->changed = true;
							front.push_back(Point3i(pt.X()+up,pt.Y()+side,pt.Z()+frontw));
						}

                        // Añadimos el incremento de llegar a esa celda desde la actual.
						weightsSum += neighborCell->data->ownerWeight;
						weightsCount++;
                    }
                }
            }

			if(weightsCount > 0)
				weightsSum = weightsSum / weightsCount;
			else
			{
				printf("Como puede ser que no haya ningun peso?.\n");
				weightsSum = 0;
			}

			cell->data->tempOwnerWeight = weightsSum;
        }

		for(unsigned int frIdx = 0; frIdx < front.size(); frIdx++)
        {
            Point3i pt = front[frIdx];
            cell3d* cell = grid.cells[pt.X()][pt.Y()][pt.Z()];
			cell->data->ownerWeight = cell->data->tempOwnerWeight;
		}

        iter++;
    }

	for(unsigned int frIdx = 0; frIdx < front.size(); frIdx++)
    {
        Point3i pt = front[frIdx];
        cell3d* cell = grid.cells[pt.X()][pt.Y()][pt.Z()];

		if(cell->data->ownerWeight < 0 || cell->data->ownerWeight > 1)
		{
			int problemas = 0;
			printf("hay algun problema en el calculo de pesos.\n");
		}

		float temp = cell->data->ownerWeight;
		if(temp != cell->data->ownerWeight )
			int ala= 0;

		cell->data->auxInfluences.push_back(weight(idFront, cell->data->ownerWeight));
	}
}

void SmoothFromSegment(grid3d& grid, int frontId)
{
    // Fronts creation using frontId
    vector< Point3i > front;

    // Init grid for weights smoothing
	printf("\n-- initCellWeightsSmoothing --\n"); fflush(0);
    initCellWeightsSmoothing(grid, front, frontId);

    // Init grid flags for front propagations.
	//printf("-- Init grid flags -- \n"); fflush(0);
    //initGridFlagsForFrontPropagation(grid, frontId);

    // Smoothing (sin corte jerárquico)
    vector<Point3i> postProcess;
	printf("-- Smoothing -- \n"); fflush(0);
	float realSmooth = grid.smoothPropagationRatio* grid.worldScale;
	int smoothingPasses = 5;

    weightsSmoothing(grid, front, realSmooth /*grRend->smoothPropagationRatio*/, frontId, smoothingPasses);
}

void PropagateFromSegment(grid3d& grid, int frontId)
{
	// TEMP TODEBUG
	SmoothFromSegment(grid, frontId);
	return;

    // Fronts creation using frontId
    vector< Point3i > front;

    // Insert in front depending on the boundaries.
	printf("\n-- Front initzialization --\n"); fflush(0);
    initfrontWithBoundaries(grid, front, frontId);

    // Init grid flags for front propagations.
	printf("-- Init grid flags -- \n"); fflush(0);
    initGridFlagsForFrontPropagation(grid, frontId);

    // Smoothing (sin corte jerárquico)
    vector<Point3i> postProcess;
	printf("-- Smoothing -- \n"); fflush(0);
	float realSmooth = grid.smoothPropagationRatio* grid.worldScale;
    borderFrontSmoothing(grid, front, postProcess, realSmooth /*grRend->smoothPropagationRatio*/, frontId);
}

void getNodesFromJointBranch(joint* jt, vector<DefNode*>& nodes, int rootBoneId)
{
	for(unsigned int i = 0; i< jt->nodes.size(); i++)
	{
		nodes.push_back(jt->nodes[i]);
		nodes.back()->rootBoneId = rootBoneId;
	}

	for(unsigned int i = 0; i < jt->childs.size(); i++)
		getNodesFromJointBranch(jt->childs[i], nodes, rootBoneId);
}

void reSegmentVolume(grid3d& grid, 
					 vector<int>& segmentationIds, 
					 vector< vector<cell3d*> >& orphanCells)
{
	// Procesamos cada grupo huérfano.
	for(unsigned int i = 0; i< orphanCells.size(); i++)
	{
		// Obtenemos los nodos que se disputarán la celda.
		int SegId = orphanCells[i][0]->data->ownerLabel;
		vector<DefNode*> newCandidateNodes;
		vector<int> newCandidates;
		for(unsigned int j = 0; j< segmentationIds.size(); j++)
		{
			if(segmentationIds[j] != SegId)
			{
				for(unsigned int skId = 0; skId < grid.bindedSkeletons.size(); skId++)
				{
					skeleton* skt = grid.bindedSkeletons[skId];
					if(skt->getJoint(segmentationIds[j]))
					{
						joint*jt = skt->getJoint(segmentationIds[j]);
						getNodesFromJointBranch(jt, newCandidateNodes, segmentationIds[j]);
					}
				}
				newCandidates.push_back(segmentationIds[j]);
			}
		}

		// Si no hay otro, nos vamos.
		if(newCandidates.size() == 0) continue;

		map<int, int> ePI;
		for(unsigned int ep = 0; ep < grid.v.embeddedPoints.size(); ep++)
		{
			ePI[grid.v.intPoints[ep].nodeId] = ep;
		}

		//Buscamos el mejor nodo por cada celda y asignamos su boneId.
		for(unsigned int cellRef = 0; cellRef < orphanCells[i].size(); cellRef++)
		{
			float distanceMax = -1;
			int nodeRefAssigned = -1;

			for(unsigned int nodeRef = 0; nodeRef < newCandidateNodes.size(); nodeRef++)
			{
					float dist = distancesFromEbeddedPointsExtended(
						grid.v.embeddedPoints[ePI[newCandidateNodes[nodeRef]->nodeId]],
						orphanCells[i][cellRef]->data->embedding, 
						newCandidateNodes[nodeRef]->expansion);	

					if(distanceMax == -1 || dist < distanceMax)
					{
						distanceMax = dist;
						nodeRefAssigned = nodeRef;
					}
			}

			if(nodeRefAssigned == -1)
			{
				printf("Tenemos un problema\n"); fflush(0);
				assert(false);
			}
			else
			{
				orphanCells[i][cellRef]->data->ownerLabel = newCandidateNodes[nodeRefAssigned]->rootBoneId;
			}
		}
	}
}

void getConnexComponents(grid3d& grid, 
						 vector<int>& segmentationIds, 
						 vector< vector<cell3d*> >& orphanCells)
{

	vector< vector< vector<cell3d*> > > connexComponents;
	connexComponents.resize(segmentationIds.size());

	for(unsigned int id = 0 ; id< segmentationIds.size(); id++)
	{	
		int idx = segmentationIds[id];
		vector<cell3d* > IdCells;
		int nComponent = -1;
		
		for(unsigned int i = 1; i < grid.cells.size()-1; i++)
		{
			for(unsigned int j = 1; j < grid.cells[i].size()-1; j++)
			{
				for(unsigned int k = 1; k < grid.cells[i][j].size()-1; k++)
				{
					if(grid.cells[i][j][k]->getType() != BOUNDARY) continue;

					cell3d* cell = grid.cells[i][j][k];
					if(cell->data->ownerLabel == idx)
					{
						cell->data->component = -1;
						cell->data->assigned = false;
						cell->data->pos = Point3i(i,j,k);
						IdCells.push_back(cell);
					}
				}
			}
		}

		vector<cell3d* > neighbours;

		// Creamos componentes conexas a partir de este id
		while(IdCells.size()>0)
		{
			cell3d* cell = IdCells.back();
			IdCells.pop_back();
			if(!cell->data->assigned)
			{
				neighbours.clear();
				nComponent = connexComponents[id].size();
				connexComponents[id].resize(connexComponents[id].size()+1);
				neighbours.push_back(cell);

				while(neighbours.size() > 0)
				{
					cell3d* cell2 = neighbours.back();
					neighbours.pop_back();

					connexComponents[id][nComponent].push_back(cell2);
					cell2->data->component = nComponent;
					cell2->data->assigned = true;

					int interval = 1;
					for( int i = -interval; i<= interval; i++)
					{
						for( int j = -interval; j<= interval; j++)
						{
							for( int k = -interval; k<= interval; k++)
							{
								if(i == 0 && j == 0 && k == 0) continue;

								int iPos = cell2->data->pos.X()+i;
								int jPos = cell2->data->pos.Y()+j;
								int kPos = cell2->data->pos.Z()+k;
								
								if(iPos < 0 || iPos > grid.dimensions.X()) continue;
								if(jPos < 0 || jPos > grid.dimensions.Y()) continue;
								if(kPos < 0 || kPos > grid.dimensions.Z()) continue;
								if(grid.cells[iPos][jPos][kPos]->getType() != BOUNDARY) continue;

								if(grid.cells[iPos][jPos][kPos]->data->assigned) continue;
								neighbours.push_back(grid.cells[iPos][jPos][kPos]);
							}
						}
					}
				}
			}
		}
	}

	vector< vector< bool > > connexComponentvalidation;
	connexComponentvalidation.resize( connexComponents.size());
	for(unsigned int i = 0; i < connexComponentvalidation.size(); i++)
	{	
		connexComponentvalidation[i].resize(connexComponents[i].size(), false);
		for(unsigned int j = 0; j < connexComponentvalidation[i].size(); j++)
		{
			bool validated = false;
			for(unsigned int k = 0; k < connexComponents[i][j].size() && !validated; k++)
			{
				float confi = connexComponents[i][j][k]->data->confidenceLevel;
				if(confi >= grid.minConfidenceLevel)
					validated = connexComponentvalidation[i][j] = true;
			}

			for(unsigned int k = 0; k < connexComponents[i][j].size(); k++)
			{
				connexComponents[i][j][k]->data->validated = validated;
			}

			if(!validated)
			{
				orphanCells.resize(orphanCells.size()+1);
				for(unsigned int k = 0; k< connexComponents[i][j].size(); k++)
				{
					orphanCells.back().push_back(connexComponents[i][j][k]);
				}
			}
		}
	}
}

// The process goes down from every root joint and triggers a family fight for the influence.  
void computeHierarchicalSkinning(grid3d& grid, joint* jt)
{
	clock_t begin, end;
	
	//DEBUG - todelete chivatos
	printf("compute Hierarchical at second and sucesive levels\n"); fflush(0);
	if (jt->childs.size() == 0) return;

	// 1. Domain initzialization for this process step
	printf("1. Domain initzialization: "); fflush(0);
	begin = clock();
	initGridForHierarchicalskinning(grid, jt->nodeId);
	end = clock();
	printf("%f segs.\n", timelapse(end,begin)); fflush(0);

	// 2. Establish every joint influence region
	printf("2. Volume segmentation");
	map<int, int> traductionTable;
	vector<int> segmentationIds;
	// 2.a. every child joint own some node ids
	printf("2.a Traduction Table\n"); fflush(0);
	begin = clock();
	for(unsigned int id = 0; id < grid.v.intPoints.size(); id++)
		traductionTable[grid.v.intPoints[id].nodeId] = grid.v.intPoints[id].boneId;

	for(unsigned int i = 0; i< jt->childs.size(); i++)
	{
		createTraductionTable(jt->childs[i], traductionTable, jt->childs[i]->nodeId);
		segmentationIds.push_back(jt->childs[i]->nodeId);
	}
	end = clock();
	printf("%f segs.\nn", timelapse(end,begin)); fflush(0);

	// 2.b. Add also the father for fighting (TODEBUG IN TEST-TIME)
	createTraductionTable(jt, traductionTable, jt->nodeId, true);
	//segmentationIds.push_back(jt->nodeId);

	// 2.c. We translate all the cells according to its node influencer.
	printf("2.c Translation\n"); fflush(0);
	traducePartialSegmentation(grid, traductionTable);
	traductionTable.clear();

	// 2.d. There are some regions that are influence cut by another regions, 
	// then this regions well be free and discussed again.
	if(bCutRegions)
	{
		printf("2.d Cutting isolated regions\n"); fflush(0);
		begin = clock();

		vector< vector<cell3d*> > orphanCells;
		getConnexComponents(grid, segmentationIds, orphanCells);	

		// reprocesar las celdas huérfanas
		reSegmentVolume(grid, segmentationIds, orphanCells);

		end = clock();
		printf("%f segs.\n", timelapse(end,begin)); fflush(0);
	}
	else
	{
		printf("2.d. (NO) Cutting isolated regions\n"); fflush(0);
	}

	// 3. Then we do some smooth between coliding regions.
	if(bPropagate)
	{
		printf("3. Segments smoothing\n"); fflush(0);
		begin = clock();
		for(unsigned int i = 0; i< segmentationIds.size(); i++)
		{
			printf("%d ", i); fflush(0);
			PropagateFromSegment(grid, segmentationIds[i]);
		}
		end = clock();
		printf("%f segs.\n", timelapse(end,begin)); fflush(0);
	}
	else
	{
		printf("3. (NO) Segments smoothing\n"); fflush(0);
	}

	// 5. Finally there is a normalization of weights based on the father's domain
	// It process auxInfluences vector limited by cell owner domain.
	if(bNormalizeByDomain)
	{
		printf("5. Normalize weights at the same level");
		begin = clock();
		grid.normalizeWeightsByDomain();
		end = clock();
		printf(" %f segs.\n", timelapse(end,begin)); fflush(0);
	}
	else
	{
		printf("5. (NO) Weights Normalization By Domain\n"); fflush(0);
	}

	//6. To continue the process we go further down in the skeleton hierarchy.	
	for(unsigned int i = 0; i< jt->childs.size(); i++)
	{
		computeHierarchicalSkinning( grid, jt->childs[i]);
	}
	
	
	/* // FOR TEST IN SUCCESSIVE STEPS.
	currentProcessGrid = grRend;
	for(int i = 0; i< jt->childs.size(); i++)
	{
		CurrentProcessJoints.push_back(jt->childs[i]);
	}

	printf("Preparado el siguiente paso.\n"); fflush(0);
	*/
	
}

void computeHierarchicalSkinning(grid3d& grid)
{
	printf("compute Hierarchical at first level\n"); fflush(0);

	// 1. Establecemos el dominio
	printf("1. Domain initzialization\n"); fflush(0);
	//initDomainForId(grRend, -1); // Al ser el primer elemento iniciamos todo a 1 pasando el id -1.
	initGridForHierarchicalskinning(grid, FIRST_ITERATION);

	// 2. Segmentamos para todos los esqueletos en la escena
	printf("2. Volume segmentation\n"); fflush(0);
	map<int, int> traductionTable;
	vector<int> segmentationIds;

    for(unsigned int id = 0; id < grid.v.intPoints.size(); id++)
		traductionTable[grid.v.intPoints[id].nodeId] = grid.v.intPoints[id].boneId;

	for(unsigned int i = 0; i< grid.bindedSkeletons.size(); i++)
	{
		// Preparamos la traduccion para el grid.
		createTraductionTable(grid.bindedSkeletons[i]->root, traductionTable, grid.bindedSkeletons[i]->root->nodeId);
		segmentationIds.push_back(grid.bindedSkeletons[i]->root->nodeId);
	}

    traducePartialSegmentation(grid, traductionTable);  
    traductionTable.clear();

	// 2.d. There are some regions that are influence cut by another regions, 
	// then this regions well be free and discussed again.
	if(bCutRegions)
	{
		printf("2.d Cutting isolated regions\n"); fflush(0);
		vector< vector<cell3d*> > orphanCells;
		getConnexComponents(grid, segmentationIds, orphanCells);	

		// reprocesar las celdas huérfanas
		reSegmentVolume(grid, segmentationIds, orphanCells);
	}
	else
	{
		printf("2.d. (NO) Cutting isolated regions\n"); fflush(0);
	}


	// 3. Smooth entre hijos cortando según el dominio
	if(bPropagate)
	{
		printf("3. Segments smoothing\n"); fflush(0);
		for(unsigned int i = 0; i< segmentationIds.size(); i++)
		{
			PropagateFromSegment(grid, segmentationIds[i]);
		}
	}
	else
	{
		printf("3. (NO) Segments smoothing\n"); fflush(0);
	}


	// 5. Normalización de pesos basados en el dominio
	// Se basa en los vectores auxInfluences.
	if(bNormalizeByDomain)
	{
		printf("5. Weights Normalization By Domain\n"); fflush(0);
		grid.normalizeWeightsByDomain();
	}
	else
	{
		printf("5. (NO) Weights Normalization By Domain\n"); fflush(0);
	}

	//CurrentProcessJoints.clear();

	for(unsigned int i = 0; i< grid.bindedSkeletons.size(); i++)
	{
		computeHierarchicalSkinning( grid, grid.bindedSkeletons[i]->root);
	}

	grid.cleanZeroInfluences();
}

void initGridForHierarchicalskinning(grid3d& grid, int domainId_init)
{
	for(unsigned int i = 1; i < grid.cells.size()-1; i++)
	{
		for(unsigned int j = 1; j < grid.cells[i].size()-1; j++)
		{
			for(unsigned int k = 1; k < grid.cells[i][j].size()-1; k++)
			{
				if(grid.cells[i][j][k]->getType() != BOUNDARY) continue;

				cell3d* cell = grid.cells[i][j][k];

				// These are the temporary data structures for this iteration.
				cell->data->auxInfluences.clear();
				cell->data->ownerLabel = -1;

				// We set the domain parameters for this iteration
				if(domainId_init < 0)
				{
					// This is the base case, there is no domain restriction.
					cell->data->domain = 1;
					cell->data->domainId = domainId_init;
				}
				else
				{
					int idx = findWeight(cell->data->influences, domainId_init);
					if(idx >= 0)
					{
						float ddfdsdfsdf = cell->data->influences[idx].weightValue;
						if(ddfdsdfsdf != cell->data->influences[idx].weightValue)
							int iiid = 0;

						// In this case there is influence over this cell from domainId node.
						cell->data->domain = cell->data->influences[idx].weightValue;

						if(cell->data->domain != cell->data->influences[idx].weightValue)
							int iiid = 0;

						cell->data->domainId = domainId_init;
					}
					else
					{
						// In this case there is NO influence over this cell from domainId node.
						cell->data->domain = 0;
						cell->data->domainId = domainId_init;						
					}
				}

				cell->data->itPass = 0;
			}
		}
	}
}

void updateSkinningWithHierarchy(grid3d& grid)
{
	clock_t begin, end;
	bool onlyBorders = true;

	if (VERBOSE)
	{
		cout << "--- HIERARCHICAL SKINNING ---" << endl;
		cout << "1. Segmentacion del volumen: ";
		begin = clock();
	}

	// SEGMENTATION (Realizamos una segmentacion y luego trataremos los ids)
	segmentVolumeWithEmbedding(grid, grid.v.intPoints, grid.v.embeddedPoints, onlyBorders);

	if (VERBOSE)
	{
		end = clock();
		cout << double(timelapse(end,begin)) << " s" << endl << "2. Computar Hierarchical skinning: ";
		begin = clock();
	}

	// Hierarchical weight computation
	computeHierarchicalSkinning(grid);

	if (VERBOSE)
	{
		end = clock();
		cout << endl << " FIN - Computar Hierarchical skinning.\n";
	}
}

void updateSkinning(grid3d& grid)
{
	clock_t begin, end;
	bool onlyBorders = true;

	if (VERBOSE)
	{
		cout << "\n--- UPDATING SKINNING ---" << endl;
		cout << "Segmentacion del volumen: ";
		begin = clock();
	}

	// SEGMENTATION
	if(grid.metricUsed)
		segmentVolumeWithEmbedding(grid, grid.v.intPoints, grid.v.embeddedPoints, onlyBorders);
	else
		segmentVolume(grid, grid.v.intPoints);

	// TRADUCE SEGMENTATION
	traduceSegmentation(grid, grid.v.traductionTable);

	// GRID RENDERER INITIALIZATION
	int boneCount = 0;
	for(unsigned int i = 0; i< grid.bindedSkeletons.size(); i++)
		boneCount += grid.bindedSkeletons[i]->getBoneCount();

	//TOREVIEW
	//grid.init(boneCount);

	if(VERBOSE)
	{
		end=clock();
		cout << double(timelapse(end,begin)) << " s"<< endl;
		cout << "Reconstruction voronoi graph:(commented) ";
		begin=clock();
	}
	// VORONOI GRAPH INIZIALIZATION
	//buildVoronoiGraph(grRend->grid,grRend->v.voronoiGraph, grRend->v.intPoints.size());

	if(VERBOSE)
	{
		end = clock();
		cout << double(timelapse(end,begin)) << " s"<< endl;
		cout << "Recalculo de pesos: ";
		begin=clock();
	}

	// WEIGHTS PROPAGATION
	assert(false); //reimplement TODEBUG
	//PropagateWeightsFromSkt(grRend->grid, skt, grRend->smoothPropagationRatio, grRend->grid.valueRange);

	if(VERBOSE)
	{
		end = clock();
		cout << double(timelapse(end,begin)) << " s"<< endl;
		cout << "--- FIN de UPDATING SKINNING ---" << endl;
	}

}

void ComputeSkining(Modelo* m, grid3d& grid)
{
	sceneUpdatingFlags* sceneUpdating = new sceneUpdatingFlags();
	sceneUpdating->all = true;

	ComputeSkining(m,grid,sceneUpdating);

}

// This fucntion deletes all the null weights derived from the computation for reduce the data
void cleanNullWeights(grid3d& grid)
{
	for(unsigned int i = 1; i < grid.cells.size()-1; i++)
	{
		for(unsigned int j = 1; j < grid.cells[i].size()-1; j++)
		{
			for(unsigned int k = 1; k < grid.cells[i][j].size()-1; k++)
			{
				if(grid.cells[i][j][k]->getType() != BOUNDARY) continue;

				cell3d* cell = grid.cells[i][j][k];

				vector<weight> auxWeights;
				for(int w = 0; w< cell->data->influences.size(); w++)
				{
					if(cell->data->influences[w].weightValue > 0)
						auxWeights.push_back(cell->data->influences[w]);
				}

				if(auxWeights.size() == 0 && cell->data->influences.size() != 0)
					printf("Hemos eliminado todos los pesos\n");

				if(auxWeights.size() < cell->data->influences.size())
				{
					// There have been a deletion, then we can recompose it.
					cell->data->influences.clear();
					for(int w = 0; w< auxWeights.size(); w++)
						cell->data->influences.push_back(auxWeights[w]);
				}

				auxWeights.clear();
			}
		}
	}
}

void ComputeSkining(Modelo* m, grid3d& grid, sceneUpdatingFlags* sceneUpdating)
{
	clock_t begin, end;

	if(!sceneUpdating) //TODEBUG - TOSOLVE
	{
			sceneUpdating = new sceneUpdatingFlags();
			sceneUpdating->all = true;
	}
	//assert(false); // He puesto así para que sea más fácil la creación de código.

	char buffer [100];
	sprintf(buffer ,"Binded Skeletons: %d, proposing nodes...", grid.bindedSkeletons.size());
	printf(buffer);printf("\n");
	grid.SafefPrintText(buffer);
	grid.SafefPrintProcessStatus(40);
	// Crearemos los nodos a partir de los huesos.
	proposeNodes(grid.bindedSkeletons, grid.v.intPoints, sceneUpdating);

	if(grid.v.intPoints.size() == 0)
	{
		printf("No hay deformadores para realizar el cálculo.\n");
	}

	sprintf(buffer ,"Proposed nodes: %d, embedding nodes...", grid.v.intPoints.size());
	printf(buffer);printf("\n");
	grid.SafefPrintText(buffer);
	grid.SafefPrintProcessStatus(45);
	
	// Creamos la tabla de traducción general.
	grid.v.traductionTable.resize(grid.v.intPoints.size());

	for(unsigned int j = 0; j< grid.v.traductionTable.size(); j++)
		grid.v.nodeIds[grid.v.intPoints[j].nodeId] = &(grid.v.intPoints[j]);

	// NODES EMBEDING COMPUTATION (Paralelizable)
	if(VERBOSE) begin=clock();

	grid.v.embeddedPoints.resize(grid.v.intPoints.size());

	vector< Point3d> auxPoints(grid.v.intPoints.size());
	for(unsigned int ap = 0; ap < auxPoints.size(); ap++)
		auxPoints[ap] = grid.v.intPoints[ap].pos;

	/*FILE* fout1 = fopen("intPOintsTemp.txt", "w");
	fprintf(fout1," Puntos:\n");
	for(int iPt = 0; iPt < auxPoints.size(); iPt++)
	{
		fprintf(fout1,"%d:(%f, %f, %f)\n",iPt,auxPoints[iPt][0], auxPoints[iPt][1],auxPoints[iPt][2]);
	}
	fclose(fout1);

	FILE* fout2 = fopen("embedding_despues.txt", "w");
	fprintf(fout2," Este es el embedding, deberia corresponder a lo que hay:\n");
	for(int iPt = 0; iPt < m->embedding.size(); iPt++)
	{
		fprintf(fout2, "%d:(\n", iPt);
		for(int emb = 0; emb < m->embedding[iPt].size(); emb++)
		{
			fprintf(fout2, " %f,",m->embedding[iPt][emb]);
		}
		fprintf(fout2, ")\n");
	}
	fclose(fout2);
	*/

	if(sceneUpdating->updateDefNodes || sceneUpdating->all)
	{
		// Calculamos el embedding de los puntos internos.
		mvc_IBD(auxPoints, grid.v.embeddedPoints, m->embedding, *m);
	
		if(VERBOSE)
		{
			end=clock();
			cout << "Nodes embedding: " << double(timelapse(end,begin)) << " s"<< endl;
		}
	}

	/*
	FILE* fout = fopen("embeddedPoints.txt", "w");
	fprintf(fout, "Puntos embedded:\n");
	for(int iPt = 0; iPt < grid.v.embeddedPoints.size(); iPt++)
	{
		fprintf(fout, "%d:(\n", iPt);
		for(int emb = 0; emb < grid.v.embeddedPoints[iPt].size(); emb++)
		{
			fprintf(fout, " %f,",grid.v.embeddedPoints[iPt][emb]);
		}
		fprintf(fout, ")\n");
	}
	fclose(fout);
	*/

	sprintf(buffer ,"computing skinning");
	printf(buffer);printf("\n");
	grid.SafefPrintText(buffer);
	grid.SafefPrintProcessStatus(50);

	// Actualizacion del skinning con los parametros actuales.
	updateSkinningWithHierarchy(grid);
	
	sprintf(buffer ,"cleaning nulls");
	printf(buffer);printf("\n");
	grid.SafefPrintText(buffer);
	grid.SafefPrintProcessStatus(70);

	cleanNullWeights(grid);

	sprintf(buffer ,"computing secondary weights");
	printf(buffer);printf("\n");
	grid.SafefPrintText(buffer);
	grid.SafefPrintProcessStatus(80);
	
	// Calculamos pesos secundarios sobre lo calculado anteriormente.
	computeSecondaryWeights(m);

	// Actualizamos el grid para renderiazar.
	//grRend->propagateDirtyness();
}

//v1 - v2
void vectorDiference(vector<double >& v1,vector<double >& v2, vector<double >& ret)
{
	if(v1.size() != v2.size()) return;

	ret.resize(v1.size());
	for(int i = 0; i< v1.size(); i++)
	{
		ret[i] = v1[i] - v2[i];
	}
}

double norm(vector<double>& v)
{
	double sum = 0;
	for(int i = 0; i< v.size(); i++) 
		sum += v[i]*v[i];
	return sqrt(sum);
}

void unit(vector<double >& v,vector<double >& unitVector)
{
	double n = norm(v);
	unitVector.resize(v.size());
	for(int i = 0; i< v.size(); i++) unitVector[i] = v[i]/n;
}

double dotProduct(vector<double >& v1,vector<double >& v2)
{
	double res = 0;
	for(int i = 0; i< v1.size(); i++)
		res += v1[i]*v2[i];
	return res;
}

void computeSecondaryWeights(Modelo* m)
{
	if(!m->grid) return;

	for(int i = 0; i < m->grid->bindedSkeletons.size(); i++)
	{
		// calculo embedding
		for(int jt = 0; jt < m->grid->bindedSkeletons[i]->joints.size(); jt++)
		{
			joint* pjt = m->grid->bindedSkeletons[i]->joints[jt];
			mvcEmbedPoint(pjt->worldPosition, pjt->embedding, m->embedding, *m);
		}

		// calculo vector diferencia para no tener que hacerlo mil veces
		for(int jt = 0; jt < m->grid->bindedSkeletons[i]->joints.size(); jt++)
		{
			joint* pjt = m->grid->bindedSkeletons[i]->joints[jt];
			pjt->childVector.resize(pjt->childs.size());

			for(int jtChild = 0; jtChild < pjt->childs.size(); jtChild++)
			{
				vectorDiference( pjt->embedding, pjt->childs[jtChild]->embedding, pjt->childVector[jtChild]);
			}
		}
	}
	
	grid3d& grid = *(m->grid);

	 // Init grid labels & weights
    for(unsigned int i = 0; i< grid.cells.size(); i++)
    {
        for(unsigned int j = 0; j< grid.cells[i].size(); j++)
        {
            for(unsigned int k = 0; k< grid.cells[i][j].size(); k++)
            {
                cell3d* cell = grid.cells[i][j][k];
                if(cell->getType() != BOUNDARY) continue;

				cell->data->auxInfluences.clear();
				for(int wIdx = 0; wIdx < cell->data->influences.size(); wIdx++)
				{
					// Buscaremos el esqueleto al que pertenece dicho label.
					joint* jtLinked = NULL;
					int i = 0; 
					while(!jtLinked && i < grid.bindedSkeletons.size())
					{
						jtLinked = grid.bindedSkeletons[i]->getJoint(cell->data->influences[wIdx].label);
						i++;
					}

					if(!jtLinked)
					{
						printf("No he encontrado el joint para calcular pesos secundarios...\n");
						continue;
					}

					for(int childLinked = 0; childLinked < jtLinked->childs.size(); childLinked++)
					{
						weight wt;
						wt.label = jtLinked->nodeId;
						wt.relativeLabel = jtLinked->childs[childLinked]->nodeId;

						vector<double> unitVector;
						unit(jtLinked->childVector[childLinked], unitVector);

						vector<double> diference;
						vectorDiference( cell->data->embedding, jtLinked->childs[childLinked]->embedding, diference);

						double projection = 0;
						if(diference.size() == 0)
						{
							printf("Tenemos un problema porque no deberia estar vacio.\n");
							printf("Para que no pete el sistema daremos un valor por defecto.\n");
						}
						else
							projection = dotProduct(unitVector, diference);

						double totalLenght = norm(jtLinked->childVector[childLinked]);

						if(projection < 0.0)
							wt.weightValue = 1.0;
						else if(projection > totalLenght)
							wt.weightValue = 0.0;
						else
							wt.weightValue = 1.0 - (projection/totalLenght);

						cell->data->auxInfluences.push_back(wt);
					}
				}
			}
		}
	}
}

 void BindSkeletons(scene* snc, grid3d* grid)
 {
	 // Hacemos una estimacion de la celda minima para dividir cualquier esqueleto.
	 for(int i = 0; i< snc->skeletons.size(); i++)
	 {
		 float cellSize = grid->cellSize;
		 GetMinSegmentLenght(getMinSkeletonSize((skeleton*)snc->skeletons[i]), cellSize);
		 ((skeleton*)snc->skeletons[i])->minSegmentLength = GetMinSegmentLenght(getMinSkeletonSize((skeleton*)snc->skeletons[i]), cellSize);

		 grid->bindedSkeletons.push_back((skeleton*)snc->skeletons[i]);
	 }
 }

 void Voxelize(scene* snc, Modelo* m, float resolution, float worldScale, bool onlyBorders)
 {
	 // Hemos vinculado el grid al modelo y esqueleto cargados.
	 grid3d* grid = m->grid;

	 //La resolución y worldScale cogidas de la interficie: parámetros a pasar.
	 grid->res = resolution;
	 grid->worldScale = worldScale;

     clock_t begin, end;

     vector< vector<double> >* embedding = NULL;
     if(m->embedding.size() != 0)
     {
         embedding = &(m->embedding);
     }
     else // No se ha cargado ningun embedding.
     {
         printf("No hay cargado ningun embedding. Lo siento, no puedo hacer calculos\n"); fflush(0);
         return;
     }

     // Cargamos el grid si ya ha sido calculado
     string sSavingFile = m->sPath + m->sName.c_str()+ "_embedded_grid.dat";
	 const char* charSavingFile = sSavingFile.c_str();
     ifstream ifile(charSavingFile);
	 if(ifile)	
     {
         if(VERBOSE)
         {
            cout << "Loading from disc: ";
            begin=clock();
         }
     
		 grid->LoadGridFromFile(sSavingFile);

         if(VERBOSE)
         {
             end = clock();
             cout << double(timelapse(end,begin)) << " s"<< endl;
         }
     }
     else
     {
         if(VERBOSE)
         {
             if(!onlyBorders) cout << "Computing volume: " << endl;
             else cout << "Computing volume (onlyAtBorders): " << endl;

             cout << "------------------------------------------------------" << endl;
             cout << "                  - PREPROCESO -                      " << endl;
             cout << "------------------------------------------------------" << endl;
             cout << "Creación del grid con res " << grid->res << endl;
             begin=clock();

         }

		char buffer[100];
		sprintf(buffer ,"Is needed to embed the grid.");
		printf(buffer);printf("\n");
		grid->SafefPrintText(buffer);
		grid->SafefPrintProcessStatus(0);

         // CONSTRUCCION DEL GRID
         gridInit(*m,*grid);

         if(VERBOSE)
         {
             end=clock();
             cout << double(timelapse(end,begin)) << " s"<< endl;
             begin = end;
             cout << "Etiquetado del grid: ";
         }

		 grid->SafefPrintProcessStatus(5);

         //Tipificamos las celdas según si es interior, exterior, o borde de la maya
         grid->typeCells(*m);

         if(VERBOSE)
         {
             end=clock();
             cout << double(timelapse(end,begin)) << " s"<< endl;
             cout << "Grid cells embedding";
             begin=clock();
         }

		 grid->SafefPrintProcessStatus(10);

         // EMBEDING INTERPOLATION FOR EVERY CELL
         int interiorCells = 0;
         interiorCells = gridCellsEmbeddingInterpolation(*m, *(grid), m->embedding, onlyBorders);

         if(VERBOSE)
         {
             end=clock();
             cout << "("<< interiorCells <<"cells): " << double(timelapse(end,begin)) << " s"<< endl;
             double memorySize = (double)(interiorCells*DOUBLESIZE*embedding[0].size())/MBSIZEINBYTES;
             cout << "Estimated memory consumming: " << memorySize << "MB" <<endl;
             cout << ">> TOTAL (construccion del grid desde cero): "<<double(timelapse(end,begin)) << " s"<< endl;

         }

         if(!sSavingFile.empty())
         {
             if(VERBOSE)
             {
                cout << "Guardando en disco: ";
                begin=clock();
             }

             grid->SaveGridToFile(sSavingFile);

             if(VERBOSE)
             {
                 clock_t end=clock();
                 cout << double(timelapse(end,begin)) << " s"<< endl;
             }
         }
     }

     //grRend->propagateDirtyness();
     //updateGridRender();
 }

void reportResults(Modelo* model)
{
	//printf("getting results %d points and %d nodes.\n",weights.points, weights.nodes);	
	int count = 0;
	float sum = 0;
	MyMesh::VertexIterator vi;

	grid3d& grid = *(model->grid);

	printf("Hola"); fflush(0); 

	FILE* fout = fopen("secondaryWeights.txt", "w");

	fprintf(fout, "Deformers...\n");
	for(int i = 0; i< grid.bindedSkeletons.size(); i++)
	{
		for(int j = 0; j< grid.bindedSkeletons[i]->joints.size(); j++)
		{
			fprintf(fout, "name:%s\n", grid.bindedSkeletons[i]->joints[j]->sName.c_str());
			fprintf(fout, "pos:%f, %f, %f\n", grid.bindedSkeletons[i]->joints[j]->pos.X()
											, grid.bindedSkeletons[i]->joints[j]->pos.Y()
											, grid.bindedSkeletons[i]->joints[j]->pos.Z());
		}
	}

	for(vi = model->vert.begin(); vi!=model->vert.end(); ++vi ) 
	{

		// Obtener celda
		Point3d ptPos = vi->P();
		Point3i pt = grid.cellId(ptPos);
		cell3d* cell = grid.cells[pt.X()][pt.Y()][pt.Z()];
		if(!cell->data)
			continue;

		fprintf(fout, "Influences: %d -> ", cell->data->influences.size()); fflush(fout);

		// copiar pesos
		for(unsigned int w = 0; w< cell->data->influences.size(); w++)
		{
			float auxWeight = cell->data->influences[w].weightValue;
			int idDeformer = cell->data->influences[w].label;
			//escena->defIds[idDeformer];
			fprintf(fout, "(%d,  %f) ", idDeformer, auxWeight); fflush(fout);
		}
		fprintf(fout, "\n"); fflush(fout);
		fprintf(fout, "Secondary Influences: %d -> ", cell->data->auxInfluences.size()); fflush(fout);
		for(unsigned int w = 0; w< cell->data->auxInfluences.size(); w++)
		{
			float auxWeight = cell->data->auxInfluences[w].weightValue;
			int idDeformer = cell->data->auxInfluences[w].label;
			int idRelDeformer = cell->data->auxInfluences[w].relativeLabel;

			//escena->defIds[idDeformer];
			fprintf(fout, "(%d,%d:%f) ", idDeformer,idRelDeformer, auxWeight); fflush(fout);
		}
		fprintf(fout, "\n\n"); fflush(fout);
		count++;
	}
}