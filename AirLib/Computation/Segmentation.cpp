#include "Segmentation.h"
#include "mvc_interiorDistances.h"

#include <DataStructures/InteriorDistancesData.h>
#include <DataStructures/skeleton.h>
#include <DataStructures/modelo.h>
#include <DataStructures/scene.h>
#include <DataStructures/SurfaceData.h>
#include <render/gridRender.h>
#include <Computation/BiharmonicDistances.h>

#define MAX_LENGTH 99999999
#define FIRST_ITERATION -1
#define MIN_CONFIDENCE_LEVEL 1.0
#define PROPAGATION_UNIT 10.0
#define VERBOSE false
#define BATCHPROCESSING false

#define bCutRegions false
#define bPropagate true
#define bNormalizeByDomain true

using namespace std;

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

////////////////////////////////////////////////////
/////////////// SEGMENTATION PROCESS ///////////////
////////////////////////////////////////////////////

void traducePartialSegmentation(Modelo& m, binding* bd, map<int, int>& traductionTable)
{
    // Recorrer cada vertice de la maya con un iterador y clasificar los vertices.
    //MyMesh::VertexIterator vi;
    //for(vi = m.vert.begin(); vi!=m.vert.end(); ++vi )
    //{
    //    Point3d pt = vi->P();
    //    int VertIdx = vi->IMark();
	for(int VertIdx = 0; VertIdx < bd->pointData.size(); VertIdx++ )
	{
        // Assegurar que los indices guardados en los vertices de la maya estan bien.
        assert(VertIdx >= 0 && VertIdx < m.vn());
        PointData& pd = bd->pointData[VertIdx];

        if(pd.segmentId>=0)
        {
            pd.ownerLabel = traductionTable[pd.segmentId];
        }
        else
        {
            printf("Hay algunas celdas visitadas que no tienen bien asignado el segmentId.[ver traduceParcialSegmentation(...)]");
        }
    }
}


// Segment volume with labels constructing a voronoi diagram
// TODEBUG -> he cambiado grid por malla.
int segmentVolumeWithEmbedding(Modelo& m, binding* bd)
{
    // m, bb->intPoints, bb->embeddedPoints
    vector< DefNode >& points = bd->intPoints;
    vector< vector<double> >& embeddedPoints = bd->embeddedPoints;
	vector< vector<int> >& sortedWeights = bd->weightsSort;
	vector<vector<weight> >& weights = bd->weightsFiltered;


	clock_t begin, end;
	double timeMean = 0;
	for(int pointId = 0; pointId < bd->pointData.size(); pointId++ )
    {
		PointData& pd = bd->pointData[pointId];

        // recorremos todos los puntos del esqueleto para realizar la segmentacion
		//float distance = distancesFromEbeddedPointsExtended(embeddedPoints[0], m.embedding[pointId], ((DefNode)points[0]).expansion);
        begin = clock();
		//float distance = BiharmonicDistanceP2P(embeddedPoints[0], pointId, bd, points[0]);
		int iniIdx = bd->globalIndirection.front();
		int finIdx = bd->globalIndirection.back();

		//float distance = BiharmonicDistanceP2P_block(embeddedPoints[0], pointId, bd, ((DefNode)points[0]).expansion, bd->intPoints[0].precomputedDistances, iniIdx, finIdx);
		float distance = 0;
		if(useMVC)
		{
			distance = -BiharmonicDistanceP2P_sorted(embeddedPoints[0], sortedWeights[0], pointId, bd, ((DefNode)points[0]).expansion, bd->intPoints[0].precomputedDistances, bd->cutThreshold[0]);
		}
		else
		{
			distance = -BiharmonicDistanceP2P_HC(weights[0], pointId, bd, ((DefNode)points[0]).expansion, bd->intPoints[0].precomputedDistances);
		}
		
		end = clock();
		timeMean += end-begin;

		int label = points[0].nodeId;

        int secondLabel = -1;
        float secondDistance = distance*99999;

        int lid = 0;
        int sid = -1;

        //pd.clear();
        for(unsigned int id = 1; id< embeddedPoints.size(); id++)
        {
            //float newDist =  distancesFromEbeddedPoints(embeddedPoints[nodeId],cell->data->embedding);
            //float newDist =  distancesFromEbeddedPointsExtended(embeddedPoints[id],m.embedding[VertIdx], points[id].expansion);
			begin = clock();
			//float newDist =  BiharmonicDistanceP2P(embeddedPoints[id], pointId, bd, points[id]);
			int iniIdx = bd->globalIndirection.front();
			int finIdx = bd->globalIndirection.back();
			//float newDist = BiharmonicDistanceP2P_block(embeddedPoints[id], pointId, bd, ((DefNode)points[id]).expansion, bd->intPoints[id].precomputedDistances, iniIdx, finIdx);
			float newDist = 0;
			if(useMVC)
			{
				newDist =-BiharmonicDistanceP2P_sorted(embeddedPoints[id], sortedWeights[id], pointId, bd, ((DefNode)points[id]).expansion, bd->intPoints[id].precomputedDistances, bd->cutThreshold[id]);
			
			}
			else
			{
				newDist =-BiharmonicDistanceP2P_HC(weights[id], pointId, bd, ((DefNode)points[id]).expansion, bd->intPoints[id].precomputedDistances);
			}
			end = clock();
			timeMean += end-begin;

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
            //else
            //{
            //    // We test if is worst than the first but better than the second
            //    if(newDist < secondDistance)
            //    {
            //        secondDistance = newDist;
            //        secondLabel = points[id].nodeId;

            //        sid = id;
            //    }
            //}
        }

        pd.segmentId = label;
        //pd->distanceToPos = distance;

        //if(distance > 0)
        //{
        //    float proportionalDistance = distancesFromEbeddedPointsExtended(embeddedPoints[lid],embeddedPoints[sid], 1.0);
        //    pd.confidenceLevel = ((secondDistance-distance)/proportionalDistance)/*/distance*/;
        //}
        //else
        //    pd.confidenceLevel = 10;

		//if(pointId%50 == 0)
		//{
		//	printf("Punto %d\n", pointId);
		//	fflush(0);
		//}
     }
	
	printf("Tiempo medio de calculo de distancias %f\n", timeMean/(bd->pointData.size()*embeddedPoints.size()));
	fflush(0);

    // valueRange references the label count.
    //grid.valueRange = points.size();
    return 0;
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

////////////////////////////////////////////////////
///////////////    KERNEL FUNCTIONS    /////////////
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
	// Es un problema de precision numerica de la maquina y la funcion.
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

////////////////////////////////////////////////////
///////////////  WEIGHTS PROPAGATION   /////////////
////////////////////////////////////////////////////
// Crea un frent de avance según el nodeId especificado.
// Si no hay ninguna celda con el ese nodoId, devuelve un frente vacío.
void initSurfaceWeightsSmoothing(Modelo& m, binding* bd, vector< int >& front, int nodeId)
{
	// recorrer todo el modelo e inicializar los datos para la transmision
	front.clear();
	//MyMesh::VertexIterator vi;
	//for(vi = m.vert.begin(); vi!=m.vert.end(); ++vi ) 
	//{
	//	int idx = vi->IMark();
	for(int idx = 0; idx < bd->pointData.size(); idx++ )
	{	
		if(bd->pointData[idx].ownerLabel != nodeId)
		{
			bd->surface.nodes[idx]->visited = false;
			bd->pointData[idx].itPass = 0;
			bd->pointData[idx].ownerWeight = 0.0;
			bd->pointData[idx].tempOwnerWeight = 0.0;
		}
		else
		{
			bd->surface.nodes[idx]->visited = true;
			bd->pointData[idx].itPass = 0;
			bd->pointData[idx].ownerWeight = 1.0;
			bd->pointData[idx].tempOwnerWeight = 1.0;
			front.push_back(idx);
		}
	}
}

// Propaga el peso a lo largo de la superficie, teniendo en cuenta una segmentación.
void weightsNoSmooting(Modelo& m, binding* bd,
                      vector< int >& front,
                      int idFront)
{
	for(unsigned int frIdx = 0; frIdx < front.size(); frIdx++)
    {
		PointData& pd = bd->pointData[front[frIdx]];
		pd.auxInfluences.push_back(weight(idFront, pd.ownerWeight));
	}
}

// Propaga el peso a lo largo de la superficie, teniendo en cuenta una segmentación.
void weightsSmoothing(Modelo& m, binding* bd,
                      vector< int >& front,
                      float smoothPropagationRatio,
                      int idFront,
					  int smoothingPasses)
{
    int iter = 0;
	int distance = 1;

	//FixedValue only for test... seems that there is no effects while the value is enough big.
	// I think that this is because it scales all the values, maybe I can use the biggest one
	// for preserving the precision. I can do it at the beginning with the biggest edge length.
	// By now, only for test i will use a big value;

	smoothPropagationRatio = 100;

	double weightsSum = 0;
	float weightsCount = 0;
    if(VERBOSE)printf("Front size: %d\n", front.size());
	while(iter < smoothingPasses)
    {
        for(unsigned int frIdx = 0; frIdx < front.size(); frIdx++)
        {
            int frontIdx = front[frIdx];
			PointData& pd = bd->pointData[frontIdx];
			GraphNode* sn = bd->surface.nodes[frontIdx];

			weightsSum = 0;
			weightsCount = 0;
            // Para cada elemento del frente realizamos la expansión.
			for(unsigned int neighboursIdx = 0; neighboursIdx < sn->connections.size(); neighboursIdx++)
            {
				GraphNode* snNeighbour =  sn->connections[neighboursIdx];
				if(!snNeighbour->visited)
				{
					snNeighbour->visited = true;
					front.push_back(snNeighbour->id);
				}

				// Distancia entre puntos... cuando tenga biharmonic distances ya estará calculado.
				Point3d vec = pd.node->position - bd->pointData[snNeighbour->id].node->position;
				float edgeDistance = vec.Norm();
				float distanceWeight = computeWeightProportional(edgeDistance, smoothPropagationRatio, true);

				// La media es ponderada
				weightsSum += bd->pointData[snNeighbour->id].ownerWeight*distanceWeight;
				weightsCount += 1*distanceWeight;
			}

			if(weightsCount > 0)
				weightsSum = weightsSum / weightsCount;
			else
			{
				//printf("Como puede ser que no haya ningun peso?.\n");
				weightsSum = 0;
			}

			pd.tempOwnerWeight = weightsSum;
        }

		for(unsigned int frIdx = 0; frIdx < front.size(); frIdx++)
        {
            PointData& pd = bd->pointData[front[frIdx]];
			pd.ownerWeight = pd.tempOwnerWeight;
		}

        iter++;
    }

	for(unsigned int frIdx = 0; frIdx < front.size(); frIdx++)
    {
		PointData& pd = bd->pointData[front[frIdx]];
		pd.auxInfluences.push_back(weight(idFront, pd.ownerWeight));
	}
}

void SmoothFromSegment(Modelo& m, binding* bd, int frontId)
{
    // Fronts creation using frontId
    vector< int > front;

    // Init grid for weights smoothing
	if(VERBOSE) printf("\n-- initCellWeightsSmoothing --\n");fflush(0);
    initSurfaceWeightsSmoothing(m, bd, front, frontId);

    // Smoothing (sin corte jerárquico)
	if(VERBOSE) printf("-- Smoothing -- \n");fflush(0);

	//float realSmooth = grid.smoothPropagationRatio* grid.worldScale;
	//int smoothingPasses = 2;
	//float realSmooth = 0.15;

	int smoothingPasses = bd->smoothingPasses;
	float realSmooth = bd->smoothPropagationRatio;

    weightsSmoothing(m, bd, front, realSmooth, frontId, smoothingPasses);
	//weightsNoSmooting(m, bd, front, frontId);
}

////////////////////////////////////////////////////
///////////////  SKINNING COMPUTATION   ////////////
////////////////////////////////////////////////////

// Precomputa distancias teniendo en cuenta que A es symetrica
// y un bloque definido entre iniIdx y finIdx. Consideramos la
// matriz de entrada como un bloque dentro de la matriz general.
double PrecomputeDistancesSingular_block(vector<double>& weights, symMatrix& BihDistances, int iniIdx, int finIdx)
{
	int size = finIdx-iniIdx+1;//BihDistances.size;
	if(size <= 0) return 0;

	double res = 0;
	int row = 0;
	int col = 0;
	for(int j = iniIdx; j< finIdx+1; j++)
	{
		double sum = 0;
		col = row+1;
		for(int k = j+1; k< size; k++)
		{
			sum += weights[k]*BihDistances.get(row,col);
			col++;
		}
		res += 2*sum*weights[j];
		row++;
	}

	return res;
}

// Precomputar distancias, teniendo en cuenta solo los pesos necesarios.
double PrecomputeDistancesSingular_sorted(symMatrix& BihDistances, vector<weight>& weights)
{
	int size = BihDistances.size;
	double res = 0;
	for(int j = 0; j< weights.size(); j++)
	{
		double sum = 0;
		for(int k = 0; k< weights.size(); k++)
		{
			int labelJ =  weights[j].label;
			int labelK =  weights[k].label;
			double auxWeight = weights[k].weightValue;
			double value = auxWeight*BihDistances.get(labelJ,labelK);
			sum += value;
		}

		res += sum*weights[j].weightValue;
	}
	return res;
}

double PrecomputeDistancesSingular_sorted(vector<double>& weights, vector<int>& indirection, symMatrix& BihDistances, double threshold)
{
	int size = BihDistances.size;
	double res = 0;
	for(int j = 0; j< size; j++)
	{
		if(weights[indirection[j]] < threshold) 
			break;

		double sum = 0;
		for(int k = 0; k< size; k++)
		{
			double value = weights[indirection[k]]*BihDistances.get(indirection[j],indirection[k]);
			
			// Si el valor que tenemos que sumar es muy pequeno, lo despreciamos.
			if(weights[indirection[k]] < threshold) 
				break;

			sum += value;
		}

		res += sum*weights[indirection[j]];
	}

	return res;
}


double PrecomputeDistancesSingular(vector<double>& weights, symMatrix& BihDistances)
{
	int size = BihDistances.size;
	double res = 0;
	for(int j = 0; j< size; j++)
	{
		double sum = 0;
		for(int k = 0; k< size; k++)
		{
			sum += weights[k]*BihDistances.get(j,k);
		}
		res += sum*weights[j];
	}

	return res;
}

void PrecomputeDistances(vector< vector<double> >& embeddedPoints, vector< vector<int> >& sortedWeights, symMatrix& BihDistances, vector< DefNode >& intPoints)
{
	double threshold = 1/pow(10.0,5);

	int size = BihDistances.size;
	for(unsigned int i = 0; i< embeddedPoints.size(); i++)
	{
		intPoints[i].precomputedDistances = PrecomputeDistancesSingular_sorted(embeddedPoints[i], sortedWeights[i], BihDistances, threshold) *-1;
		/*vector<double>& weights = embeddedPoints[i];
		intPoints[i].precomputedDistances = 0;
		for(int j = 0; j< size; j++)
		{
			double sum = 0;
			for(int k = 0; k< size; k++)
			{
				sum += weights[k]*BihDistances.get(j,k);
			}
			intPoints[i].precomputedDistances += sum*weights[j];
		}
		*/
	}
}


void insertInThisPos(vector<double>& valuesOrdered, vector<int>& weightsIndirection, int element, int ff,double value,int& count)
{
	double tempValue = 0;
	int tempIndirection = 0;
	for(int i = ff; i<= count; i++)
	{
		if(i == count)
		{
			valuesOrdered[i] = value;
			weightsIndirection[i] = element;
			count++;
			return;
		}

		tempValue = valuesOrdered[i];
		tempIndirection = weightsIndirection[i];

		valuesOrdered[i] = value;
		weightsIndirection[i] = element;

		value = tempValue;
		element = tempIndirection;
	}
}

void doubleArrangeElements_wS_fast(vector<double>& weights, vector<int>& orderedIndirection, double& threshold)
{
// Ordena los valores en una indireccion de mayor a menor, de manera
	// que podamos despreciar parte del calculo si el valor es muy pequeno
	orderedIndirection.resize(weights.size());
	vector<double> valuesOrdered(weights.size(), -99999);
	int count = 0;
		
	// Staticical analisis
	double minValue = 9999, maxValue = -9999, sumValues = 0;
	int countNegValues = 0;
	double sumNegValues = 0;
		
	for(unsigned int g = 0; g < weights.size(); g++)
	{
		double value = weights[g];
		
		minValue = min(minValue, value);
		maxValue = max(maxValue, value);

		if(value < 0)
		{
			countNegValues++;
			sumNegValues += value;
		}

		for(int ff = 0; ff <= count; ff++)
		{
			if(ff == weights.size())
				break;

			if(ff == count)
			{
				valuesOrdered[ff] = value;
				orderedIndirection[ff] = g;
				count++;
				break;
			}

			else if(valuesOrdered[ff] < value)
			{
				insertInThisPos(valuesOrdered, orderedIndirection, g, ff, value, count);
				break;
			}
		}

	}

	// Como este no producia resultados, lo he cambiado por 
	// anular los valores negativos con la misma cantidad de positivos.
	if(threshold == 1)
	{
		double negValueSumPos = fabs(sumNegValues);
		double sumSecondary = 0;
		int g = valuesOrdered.size()-1;
		for(; g >= 0; g--)
		{
			if(valuesOrdered[g] > 0)
			{
				sumSecondary+= valuesOrdered[g];
				if(sumSecondary >= negValueSumPos)
				{
					g++;
					break;
				}
			}
		}

		if(g > 0)
		{
			if( g == valuesOrdered.size())
				threshold = -10;
			else
				threshold = valuesOrdered[g];
		}
		else
			threshold = -10;
	}
}

void doubleArrangeElements_withStatistics(vector<double>& weights, vector<int>& orderedIndirection, vector<double>& statistics, double& threshold)
{
	// Ordena los valores en una indireccion de mayor a menor, de manera
	// que podamos despreciar parte del calculo si el valor es muy pequeno
	orderedIndirection.resize(weights.size());
	vector<double> valuesOrdered(weights.size(), -99999);
	int count = 0;
		
	// Staticical analisis
	double minValue = 9999, maxValue = -9999, sumValues = 0;
	int countOutValues = 0;
	int countNegValues = 0;
	double sumOutValues = 0;
	double sumNegValues = 0;
	double sumPosValues = 0;
		
	for(unsigned int g = 0; g < weights.size(); g++)
	{
		double value = weights[g];
		
		minValue = min(minValue, value);
		maxValue = max(maxValue, value);
		sumValues += value;
		if(value < threshold )
		{
			countOutValues++;
			sumOutValues += value;
		}
		if(value < 0)
		{
			countNegValues++;
			sumNegValues += value;
		}
		else
		{
			sumPosValues += value;
		}

		for(int ff = 0; ff <= count; ff++)
		{
			if(ff == weights.size())
				break;

			if(ff == count)
			{
				valuesOrdered[ff] = value;
				orderedIndirection[ff] = g;
				count++;
				break;
			}

			else if(valuesOrdered[ff] < value)
			{
				insertInThisPos(valuesOrdered, orderedIndirection, g, ff, value, count);
				break;
			}
		}

	}

	// Como este no producia resultados, lo he cambiado por 
	// anular los valores negativos con la misma cantidad de positivos.
	if(threshold == 1)
	{
		double negValueSumPos = fabs(sumNegValues);
		double sumSecondary = 0;
		int g = valuesOrdered.size()-1;
		for(; g >= 0; g--)
		{
			if(valuesOrdered[g] > 0)
			{
				sumSecondary+= valuesOrdered[g];
				if(sumSecondary >= negValueSumPos)
				{
					g++;
					break;
				}
			}
		}

		if(g > 0)
		{
			if( g == valuesOrdered.size())
				threshold = -10;
			else
				threshold = valuesOrdered[g];
		}
		else
			threshold = -10;
	}

	statistics.resize(8);
	statistics[0] = weights.size();
	statistics[1] = minValue;
	statistics[2] = maxValue;
	statistics[3] = sumValues;
	statistics[4] = countOutValues;
	statistics[5] = sumOutValues;
	statistics[6] = countNegValues;
	statistics[7] = sumNegValues;

	printf("\n\n--------------------------------\n");
	printf("Estadisticas del punto con %d pesos\n", weights.size());fflush(0);
	printf("Min Value: %f\n", minValue);fflush(0);
	printf("Max Value: %f\n", maxValue);fflush(0);
	printf("Sum: %f\n", sumValues);fflush(0);
	printf("Values under %f: %d: ->%f por ciento.\n", threshold, countOutValues, (float)countOutValues/(float)weights.size()*100);fflush(0);
	printf("Suman %f -> %f por ciento de la suma total.\n", sumOutValues, (float)sumOutValues/(float)sumValues*100);fflush(0);
	printf("Negative Values: %d: ->%f por ciento.\n", countNegValues, (float)countNegValues/(float)weights.size()*100);fflush(0);
	printf("Suman %f -> %f por ciento de la suma total.\n", sumNegValues, (float)sumNegValues/(float)sumValues*100);fflush(0);
	printf("------------------------------------\n\n");

}

void doubleArrangeElements(vector<double>& weights, vector<int>& orderedIndirection, bool printStatistics, double& threshold)
{
	// Ordena los valores en una indireccion de mayor a menor, de manera
	// que podamos despreciar parte del calculo si el valor es muy pequeno
	orderedIndirection.resize(weights.size());
	vector<double> valuesOrdered(weights.size(), -99999);
	int count = 0;
		
	// Staticical analisis
	double minValue = 9999, maxValue = -9999, sumValues = 0;
	int countOutValues = 0;
	int countNegValues = 0;
	double sumOutValues = 0;
	double sumNegValues = 0;
		
	for(unsigned int g = 0; g < weights.size(); g++)
	{
		double value = weights[g];

		minValue = min(minValue, value);
		maxValue = max(maxValue, value);

		if(printStatistics)
		{
			sumValues += value;
			if(value < threshold )
			{
				countOutValues++;
				sumOutValues += value;
			}
			if(value < 0)
			{
				countNegValues++;
				sumNegValues += value;
			}
		}

		for(int ff = 0; ff <= count; ff++)
		{
			if(ff == weights.size())
				break;

			if(ff == count)
			{
				valuesOrdered[ff] = value;
				orderedIndirection[ff] = g;
				count++;
				break;
			}

			else if(valuesOrdered[ff] < value)
			{
				insertInThisPos(valuesOrdered, orderedIndirection, g, ff, value, count);
				break;
				}
		}

	}

	if(threshold == 0)
	{
		double valueDiference = maxValue - minValue;
		threshold = valueDiference*0.20 + minValue;
	}
	

	if(printStatistics)
	{
		printf("\n\n--------------------------------\n");
		printf("Estadisticas del punto con %d pesos\n", weights.size());fflush(0);
		printf("Min Value: %f\n", minValue);fflush(0);
		printf("Max Value: %f\n", maxValue);fflush(0);
		printf("Sum: %f\n", sumValues);fflush(0);
		printf("Values under %f: %d: ->%f por ciento.\n", threshold, countOutValues, (float)countOutValues/(float)weights.size()*100);fflush(0);
		printf("Suman %f -> %f por ciento de la suma total.\n", sumOutValues, (float)sumOutValues/(float)sumValues*100);fflush(0);
		printf("Negative Values: %d: ->%f por ciento.\n", countNegValues, (float)countNegValues/(float)weights.size()*100);fflush(0);
		printf("Suman %f -> %f por ciento de la suma total.\n", sumNegValues, (float)sumNegValues/(float)sumValues*100);fflush(0);
		printf("------------------------------------\n\n");
	}

}

void clearOlderComputations(Modelo& m)
{
	// Clear all older computed influences
	for(int bind = 0; bind< m.bindings.size(); bind++)
	{
		for(int ptIdx = 0; ptIdx< m.bindings[bind]->pointData.size(); ptIdx++)
		{
			m.bindings[bind]->pointData[ptIdx].influences.clear();
			m.bindings[bind]->pointData[ptIdx].secondInfluences.clear();
		}
	}

	//TODEBUG:
	// Other actions for cleaning and setting up the data.
}

// update: 10 de junio de 2013
// Computacion del skinning usando directamente la maya... nada de grid
// Solo procesamos el binding que entra como parametro
void ComputeSkining(Modelo& m)
{
	clock_t ini = clock();
	clock_t begin, end;

	if(m.bindings.size() <= 0) return; 

	//TODO: We are taking only one binding
	binding* bb = m.bindings[0];
	
	if(VERBOSE)
		printf("Binded Skeletons: %d\n", bb->bindedSkeletons.size());fflush(0);

	// Crearemos los nodos a partir de los huesos.
    proposeNodes(bb->bindedSkeletons, bb->intPoints);
    
	if(VERBOSE)
		printf("Proposed nodes: %d\n", bb->intPoints.size());fflush(0);

	// Creamos la tabla de traducción general.
    bb->traductionTable.resize(bb->intPoints.size());

    for(unsigned int j = 0; j< bb->traductionTable.size(); j++)
        bb->nodeIds[bb->intPoints[j].nodeId] = &(bb->intPoints[j]);

	// NODES EMBEDING COMPUTATION (Paralelizable)
	if(VERBOSE) begin=clock();

    bb->embeddedPoints.resize(bb->intPoints.size());

    vector< Point3d> auxPoints(bb->intPoints.size());
	for(unsigned int ap = 0; ap < auxPoints.size(); ap++)
        auxPoints[ap] = bb->intPoints[ap].pos;

	/*
	// LECTURA EXTERNA DE PUNTOS INTERIORES
	FILE* fout1 = fopen("intPOintsTemp.txt", "w");
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

	// Calculamos el embedding de los puntos internos.
    
	bb->weightsSort.resize(auxPoints.size());
	bb->weightsRepresentative.resize(auxPoints.size());
	//double threshold = 1/pow(10.0, 5);

	if(!useMVC) bb->weightsFiltered.resize(auxPoints.size());

	// Cargamos el threshold especificado. En el caso de ser 1
	// será sustitido para cada punto por el threshold oportuno.
	double threshold = bb->weightsCutThreshold;
	vector<double>& currentThreshold = bb->cutThreshold;
	currentThreshold.resize(auxPoints.size(), threshold); 

	clock_t fin = clock();
	printf("Preparacion de esqueletos... para computar pesos: %f s.\n", double(timelapse(fin,ini)));

	ini = clock();
	for(unsigned int i = 0; i< auxPoints.size(); i++)
    {
		// MVC
		if(useMVC)
		{
			mvcAllBindings(auxPoints[i], bb->embeddedPoints[i], m.bindings, m);
			doubleArrangeElements_wS_fast(bb->embeddedPoints[i],bb->weightsSort[i], currentThreshold[i]);
		}
		else
		{
			//m.HCgrid->getCoordsFromPoint(auxPoints[i], bb->weightsFiltered[i]);

			//loat sum001 = 0;
			//for(int sumIdx = 0; sumIdx < bb->weightsFiltered[i].size(); sumIdx++)
			//{
			//	sum001 += bb->weightsFiltered[i][sumIdx].weightValue;
			//}

			//vector<weight> weightsTemp;
			m.HCgrid->getCoordsFromPointSimple(auxPoints[i], bb->weightsFiltered[i]);
			//int temp = 0;*/
		}

		/*
		// CALCULOS VIEJOS.
		//vector<double> stats;
		//doubleArrangeElements_withStatistics(bb->embeddedPoints[i],bb->weightsSort[i], stats, currentThreshold[i]);
		//doubleArrangeElements(bb->embeddedPoints[i],bb->weightsSort[i], false, bb->weightsCutThreshold);
		// cambiamos el signo para el calculo posterior
		//for(int ebpV = 0; ebpV < bb->embeddedPoints[i].size(); ebpV++)
		//	bb->embeddedPoints[i][ebpV] = -bb->embeddedPoints[i][ebpV];
		*/

	}
	fin = clock();

	printf("Weights Threshold Computed\n");fflush(0);

	FILE* foutLog = fopen((string(DIR_MODELS_FOR_TEST) + "outLog.txt").c_str(), "a");
	fprintf(foutLog, "%f\n", double(timelapse(fin,ini)));
	fclose(foutLog);

	//mvc_weights(auxPoints, bb->embeddedPoints, bb, m);
    //mvc_embedding(auxPoints, bb->embeddedPoints, m.embedding, m);

	// hasta aquí todo es comun, o lo consideramos asi. Ahora copiamos lo calculado.
	for(unsigned int i = 1; i< m.bindings.size(); i++)
	{
		// la primera parte la copio tal cual para simplificar... me imagino que no sera
		// asi en el caso de asignar huesos a cada parte para simplificar los calculos.
		binding* otherbb = m.bindings[i];
		proposeNodes(otherbb->bindedSkeletons, otherbb->intPoints);
		// Creamos la tabla de traducción general.
		otherbb->traductionTable.resize(otherbb->intPoints.size());

		for(unsigned int j = 0; j< otherbb->traductionTable.size(); j++)
			otherbb->nodeIds[otherbb->intPoints[j].nodeId] = &(otherbb->intPoints[j]);

		// Los pesos si los copio porque prefiero ahorrar en estos calculos.
		otherbb->embeddedPoints.resize(otherbb->intPoints.size());
		for(int ebp = 0; ebp < bb->embeddedPoints.size(); ebp++)
		{
			otherbb->embeddedPoints[ebp].resize(bb->embeddedPoints[ebp].size());
			for(int ebpV = 0; ebpV < bb->embeddedPoints[ebp].size(); ebpV++)
			{
				otherbb->embeddedPoints[ebp][ebpV] = bb->embeddedPoints[ebp][ebpV];
			}
		}
	}

	if(VERBOSE)
	{
		end=clock();
		cout << "Nodes embedding: " << double(timelapse(end,begin)) << " s"<< endl;
		begin = clock();
	}


	ini = clock();
	// Calculamos al precomputacion como una suma, suponiendo que todos los puntos van con todos... uhmmm
	vector<float> preComputedDistancesSum;
	preComputedDistancesSum.resize(m.bindings[0]->intPoints.size(), 0);
	for(int bbIdx = 0; bbIdx < m.bindings.size(); bbIdx++)
	{
		binding* otherbb = m.bindings[bbIdx];
		for(int i = 0; i< otherbb->embeddedPoints.size(); i++)
		{
			int iniIdx = otherbb->globalIndirection.front();
			int finIdx = otherbb->globalIndirection.back();
			
			// CALCULOS ANTERIORES
			//preComputedDistancesSum[i] -= PrecomputeDistancesSingular_block(otherbb->embeddedPoints[i], otherbb->BihDistances, iniIdx, finIdx);
			//preComputedDistancesSum[i] += PrecomputeDistancesSingular(otherbb->embeddedPoints[i], otherbb->BihDistances);
			
			//MVC
			if(useMVC)
			{
				preComputedDistancesSum[i] += PrecomputeDistancesSingular_sorted(otherbb->embeddedPoints[i], otherbb->weightsSort[i], otherbb->BihDistances, currentThreshold[i]);
			}
			else
			{
				preComputedDistancesSum[i] += PrecomputeDistancesSingular_sorted(otherbb->BihDistances, bb->weightsFiltered[i]);
			}

			if(threshold != currentThreshold[i] )
			{
				printf("This threshold has been changed: %f\n", currentThreshold[i]);
				fflush(0);
			}

			//preComputedDistancesSum[i] += PrecomputeDistancesSingular_sorted(otherbb->embeddedPoints[i], otherbb->weightsSort[i], otherbb->BihDistances, otherbb->weightsCutThreshold);
		}
	}

	for(unsigned int bbIdx = 0; bbIdx < m.bindings.size(); bbIdx++)
	{
		binding* otherbb = m.bindings[bbIdx];
		for(unsigned int i = 0; i< otherbb->embeddedPoints.size(); i++)
		{
			int iniIdx = otherbb->globalIndirection.front();
			int finIdx = otherbb->globalIndirection.back();
			otherbb->intPoints[i].precomputedDistances = preComputedDistancesSum[i];
			//otherbb->intPoints[i].precomputedDistances = PrecomputeDistancesSingular_block(otherbb->embeddedPoints[i], otherbb->BihDistances, iniIdx, finIdx);
			//bb->intPoints[i].precomputedDistances = PrecomputeDistancesSingular(bb->embeddedPoints[i], bb->BihDistances) *-1;
		}
	}

	//PrecomputeDistances(bb->embeddedPoints, bb->BihDistances, bb->intPoints);
	
	if(VERBOSE)
	{
		end=clock();
		cout << "Precomputed Distances: " << double(timelapse(end,begin)) << " s for "<< bb->intPoints.size()<< " points" << endl;
		fflush(0);
	}

	fin = clock();
	foutLog = fopen((string(DIR_MODELS_FOR_TEST) + "outLog.txt").c_str(), "a");
	fprintf(foutLog, "%f\n", double(timelapse(fin,ini)));
	fclose(foutLog);

	printf("Mean: %f s.\n", double(timelapse(fin,ini))/auxPoints.size());

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

	// Actualizacion del skinning con los parametros actuales.
	//for(int i = 0; i< m.bindings.size(); i++)
	updateSkinningWithHierarchy(m);

	// Calculamos pesos secundarios sobre lo calculado anteriormente.
    computeSecondaryWeights(&m);

	// Actualizamos el grid para renderiazar.
	//grRend->propagateDirtyness();
}

// The process goes down from every root joint and triggers a family fight for the influence.  
void computeHierarchicalSkinning(Modelo &m, binding* bb, joint* jt)
{
	clock_t begin, end;
	
	//DEBUG - todelete chivatos
	if(VERBOSE)printf("compute Hierarchical at second and sucesive levels\n");fflush(0);
	if (jt->childs.size() == 0) return;

	// 1. Domain initzialization for this process step
	if(VERBOSE)printf("1. Domain initzialization: ");fflush(0);
	if(VERBOSE) begin = clock();

    initDomain(m, bb, jt->nodeId);
    //initGridForHierarchicalskinning(m, jt->nodeId);

    if(VERBOSE) end = clock();
	if(VERBOSE)printf("%f segs.\n", timelapse(end,begin));fflush(0);

	// 2. Establish every joint influence region
	if(VERBOSE)printf("2. Volume segmentation");
	map<int, int> traductionTable;
	vector<int> segmentationIds;

	// 2.a. every child joint own some node ids
	if(VERBOSE)printf("2.a Traduction Table\n");fflush(0);
	if(VERBOSE) begin = clock();
    for(unsigned int id = 0; id < bb->intPoints.size(); id++)
        traductionTable[bb->intPoints[id].nodeId] = bb->intPoints[id].boneId;

	for(unsigned int i = 0; i< jt->childs.size(); i++)
	{
		createTraductionTable(jt->childs[i], traductionTable, jt->childs[i]->nodeId);
		segmentationIds.push_back(jt->childs[i]->nodeId);
	}
	if(VERBOSE) end = clock();
	if(VERBOSE)printf("%f segs.\nn", timelapse(end,begin));fflush(0);

	// 2.b. Add also the father for fighting (TODEBUG IN TEST-TIME)
	createTraductionTable(jt, traductionTable, jt->nodeId, true);
	//segmentationIds.push_back(jt->nodeId);

	// 2.c. We translate all the cells according to its node influencer.
	if(VERBOSE)printf("2.c Translation\n");fflush(0);
    traducePartialSegmentation(m, bb, traductionTable);
	traductionTable.clear();

	// 2.d. There are some regions that are influence cut by another regions, 
	// then this regions well be free and discussed again.
	if(bCutRegions)
	{
        assert(false);
        // Hay que reimplementar esta parte porque ya no hay grid.
        /*
		printf("2.d Cutting isolated regions\n");fflush(0);
		begin = clock();

		vector< vector<cell3d*> > orphanCells;
        getConnexComponents(m, segmentationIds, orphanCells);

		// reprocesar las celdas huérfanas
        reSegmentVolume(m, segmentationIds, orphanCells);

		end = clock();
		printf("%f segs.\n", timelapse(end,begin));fflush(0);
        */
	}
	else
	{
		if(VERBOSE)printf("2.d. (NO) Cutting isolated regions\n");fflush(0);
	}

	// 3. Then we do some smooth between coliding regions.
	if(bPropagate)
	{
		if(VERBOSE)
		{
			printf("3. Segments smoothing\n");fflush(0);
			begin = clock();
			for(unsigned int i = 0; i< segmentationIds.size(); i++)
			{
				printf("%d ", i);fflush(0);
				SmoothFromSegment(m, bb, segmentationIds[i]);
			}
			end = clock();
			printf("%f segs.\n", timelapse(end,begin));fflush(0);
		}
		else
		{
			for(unsigned int i = 0; i< segmentationIds.size(); i++)
				SmoothFromSegment(m, bb, segmentationIds[i]);
		}
	}
	else
	{
		if(VERBOSE)printf("3. (NO) Segments smoothing\n");fflush(0);
	}

	// 5. Finally there is a normalization of weights based on the father's domain
	// It process auxInfluences vector limited by cell owner domain.
	if(bNormalizeByDomain)
	{
		if(VERBOSE)printf("5. Normalize weights at the same level");
		if(VERBOSE)begin = clock();
        normalizeWeightsByDomain(bb);
		if(VERBOSE)end = clock();
		if(VERBOSE)printf("%f segs.\n", timelapse(end,begin));fflush(0);
	}
	else
	{
		printf("5. (NO) Weights Normalization By Domain\n");fflush(0);
	}

	//6. To continue the process we go further down in the skeleton hierarchy.	
	for(unsigned int i = 0; i< jt->childs.size(); i++)
	{
        computeHierarchicalSkinning( m, bb, jt->childs[i]);
	}
}

void computeHierarchicalSkinning(Modelo &m, binding* bb)
{
	if(VERBOSE)printf("compute Hierarchical at first level\n");fflush(0);

	clearOlderComputations(m);

	// 1. Establecemos el dominio
	if(VERBOSE)printf("1. Domain initzialization\n");fflush(0);
	//initDomainForId(grRend, -1); // Al ser el primer elemento iniciamos todo a 1 pasando el id -1.
    //initGridForHierarchicalskinning(grid, FIRST_ITERATION);
    initDomain(m, bb, FIRST_ITERATION);

	// 2. Segmentamos para todos los esqueletos en la escena
	if(VERBOSE)printf("2. Volume segmentation\n");fflush(0);
	map<int, int> traductionTable;
	vector<int> segmentationIds;

    for(unsigned int id = 0; id < bb->intPoints.size(); id++)
        traductionTable[bb->intPoints[id].nodeId] = bb->intPoints[id].boneId;

    for(unsigned int i = 0; i< bb->bindedSkeletons.size(); i++)
	{
		// Preparamos la traduccion para el grid.
        createTraductionTable(bb->bindedSkeletons[i]->root, traductionTable, bb->bindedSkeletons[i]->root->nodeId);
        segmentationIds.push_back(bb->bindedSkeletons[i]->root->nodeId);
	}

    traducePartialSegmentation(m, bb, traductionTable);
    traductionTable.clear();

	// 2.d. There are some regions that are influence cut by another regions, 
	// then this regions well be free and discussed again.
	if(bCutRegions)
	{
        assert(false);
        // TODEBUG...

        /*
		printf("2.d Cutting isolated regions\n");fflush(0);
		vector< vector<cell3d*> > orphanCells;
		getConnexComponents(grid, segmentationIds, orphanCells);	

		// reprocesar las celdas huérfanas
		reSegmentVolume(grid, segmentationIds, orphanCells);
        */
	}
	else
	{
		if(VERBOSE)printf("2.d. (NO) Cutting isolated regions\n");fflush(0);
	}


	// 3. Smooth entre hijos cortando según el dominio
	if(bPropagate)
	{
		if(VERBOSE){printf("3. Segments smoothing\n");fflush(0);}
		for(unsigned int i = 0; i< segmentationIds.size(); i++)
		{
            SmoothFromSegment(m, bb, segmentationIds[i]);
		}
	}
	else
	{
		if(VERBOSE)printf("3. (NO) Segments smoothing\n");fflush(0);
	}


	// 5. Normalización de pesos basados en el dominio
	// Se basa en los vectores auxInfluences.
	if(bNormalizeByDomain)
	{
		if(VERBOSE)printf("5. Weights Normalization By Domain\n");fflush(0);
        normalizeWeightsByDomain(bb);
	}
	else
	{
		if(VERBOSE)printf("5. (NO) Weights Normalization By Domain\n");fflush(0);
	}

	//CurrentProcessJoints.clear();

    for(unsigned int i = 0; i< bb->bindedSkeletons.size(); i++)
	{
        computeHierarchicalSkinning( m, bb, bb->bindedSkeletons[i]->root);
	}

    cleanZeroInfluences(bb);
}


void initDomain(Modelo& m, binding* bd, int domainId_init)
{
    //MyMesh::VertexIterator vi;
    //for(vi = m.vert.begin(); vi!=m.vert.end(); ++vi )
    //{
    //    Point3d pt = vi->P();
    //    int VertIdx = vi->IMark();

        // Assegurar que los indices guardados en los vertices de la maya estan bien.
    //    assert(VertIdx >= 0 && VertIdx < m.vn);
	for(unsigned int VertIdx = 0; VertIdx < bd->pointData.size(); VertIdx++ )
	{
		PointData& pd = bd->pointData[VertIdx];

        // These are the temporary data structures for this iteration.
        pd.auxInfluences.clear();
        pd.ownerLabel = -1;

        // We set the domain parameters for this iteration
        if(domainId_init < 0)
        {
            // This is the base case, there is no domain restriction.
            pd.domain = 1;
            pd.domainId = domainId_init;
        }
        else
        {
            int idx = findWeight(pd.influences, domainId_init);
            if(idx >= 0)
            {
                // In this case there is influence over this cell from domainId node.
                pd.domain = pd.influences[idx].weightValue;
                pd.domainId = domainId_init;
            }
            else
            {
                // In this case there is NO influence over this cell from domainId node.
                pd.domain = 0;
                pd.domainId = domainId_init;
            }
        }

        pd.itPass = 0;
    }
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

/*
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
				printf("Tenemos un problema\n");fflush(0);
				assert(false);
			}
			else
			{
				orphanCells[i][cellRef]->data->ownerLabel = newCandidateNodes[nodeRefAssigned]->rootBoneId;
			}
		}
	}
*/
}



void getConnexComponents(grid3d& grid, 
						 vector<int>& segmentationIds, 
						 vector< vector<cell3d*> >& orphanCells)
{
	/*
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
	*/
}



void updateSkinningWithHierarchy(Modelo&m)
{
	clock_t begin, end;

	//if (VERBOSE)
	//{
		cout << "--- HIERARCHICAL SKINNING ---" << endl;
		cout << "1. Segmentacion del volumen: ";
		begin = clock();
		fflush(0);
	//}

	clock_t ini = clock();

	// SEGMENTATION (Realizamos una segmentacion y luego trataremos los ids)
	for(unsigned int i = 0; i< m.bindings.size(); i++)
		segmentVolumeWithEmbedding(m, m.bindings[i]);
	
	clock_t fin;

	if(BATCHPROCESSING)
	{
		fin = clock();
		FILE* foutLog = fopen((string(DIR_MODELS_FOR_TEST) + "outLog.txt").c_str(), "a");
		fprintf(foutLog, "%f\n", double(timelapse(fin,ini)));fflush(0);
		fclose(foutLog);
	}

	//if (VERBOSE)
	//{
		end = clock();
		cout << double(timelapse(end,begin)) << " s" << endl << "2. Computar Hierarchical skinning: ";
		begin = clock();
		fflush(0);
	//}

	ini = clock();
	// Hierarchical weight computation
	for(unsigned int i = 0; i< m.bindings.size(); i++)
		computeHierarchicalSkinning(m, m.bindings[i]);

	if(BATCHPROCESSING)
	{
		fin = clock();
		FILE* foutLog = fopen((string(DIR_MODELS_FOR_TEST) + "outLog.txt").c_str(), "a");
		fprintf(foutLog, "%f\n", double(timelapse(fin,ini)));fflush(0);
		fclose(foutLog);
	}

	//if(VERBOSE)
	//{
		end = clock();
		cout << endl << " FIN - Computar Hierarchical skinning.\n";
		fflush(0);
	//}
}

void updateSkinning(Modelo& m)
{
    assert(false);
    //TOEDIT: al haber hecho cambios tendria que adaptarlo

    /*
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

    */

}


//v1 - v2
void vectorDiference(vector<double >& v1,vector<double >& v2, vector<double >& ret)
{
	if(v1.size() != v2.size()) return;

	ret.resize(v1.size());
	for(unsigned int i = 0; i< v1.size(); i++)
	{
		ret[i] = v1[i] - v2[i];
	}
}

double norm(vector<double>& v)
{
	double sum = 0;
	for(unsigned int i = 0; i< v.size(); i++) 
		sum += v[i]*v[i];
	return sqrt(sum);
}

void unit(vector<double >& v,vector<double >& unitVector)
{
	double n = norm(v);
	unitVector.resize(v.size());
	for(unsigned int i = 0; i< v.size(); i++) unitVector[i] = v[i]/n;
}

double dotProduct(vector<double >& v1,vector<double >& v2)
{
	double res = 0;
	for(int i = 0; i< v1.size(); i++)
		res += v1[i]*v2[i];
	return res;
}

int indexOfNode(int nodeId, vector<DefNode>& nodes)
{
	for(int i = 0; i< nodes.size(); i++)
	{
		if(nodes[i].nodeId == nodeId) return i;
	}

	return -1;
}

void computeSecondaryWeights(Modelo* m)
{
    // No hay ningun binding
    if(m->bindings.size() == 0 )
        return;

	for(int bind = 0; bind < m->bindings.size(); bind++)
	{
		binding* bd =  m->bindings[bind];
		vector< DefNode >& points = bd->intPoints;
		vector< vector<double> >& weights = bd->embeddedPoints;
		vector< vector<int> >& sortedWeights = bd->weightsSort;
		vector< vector<weight> >& weightsClean = bd->weightsFiltered; 
		
		printf("Calculo de pesos secundarios del binding %d: \n", bind);fflush(0);
		for(int pt = 0; pt < bd->pointData.size(); pt++)
		{
			if(pt%300 == 0)
			{
				printf("%fp.\n", (float)pt/(float)bd->pointData.size());
				fflush(0);
			}

			PointData& dp = bd->pointData[pt];
			dp.secondInfluences.resize(dp.influences.size());
			for(int infl = 0; infl< dp.influences.size(); infl++)
			{
				int idInfl = dp.influences[infl].label;

				// 1. Buscamos el joint de influencia
				joint* jt = NULL;
				for(int sktId = 0; sktId < bd->bindedSkeletons.size(); sktId++)
				{
					jt = bd->bindedSkeletons[sktId]->getJoint(idInfl);
					if(jt) break;
				}

				if(!jt)
				{
					printf("Debería existir la influencia...!\n");fflush(0);
				}

				dp.secondInfluences[infl].resize(jt->childs.size(), 0.0);
				// Caso base: 1 solo joint -> no guardamos pesos secundarios porque no tiene hijo

				for(int childIdx = 0; childIdx < jt->childs.size(); childIdx++)
				{
					// Recogemos los nodos relevantes.
					vector<DefNode*> relevantNodes;
					for(int candidateNodes = 0; candidateNodes < jt->nodes.size(); candidateNodes++)
					{
						if(jt->nodes[candidateNodes]->childBoneId < 0)
						{
							relevantNodes.push_back(jt->nodes[candidateNodes]);
							continue;
						}
						else if(jt->nodes[candidateNodes]->childBoneId == jt->childs[childIdx]->nodeId)
						{
							relevantNodes.push_back(jt->nodes[candidateNodes]);
						}
					}
					// Debería haber al menos 1 nodo.
					// Lo empilamos para evaluar todo el segmento.
					//relevantNodes.push_back(jt->childs[childIdx]->nodes[0]);

					double thresh = -10;// bd->weightsCutThreshold;
				
					float bestDistance = 9999999;
					int bestNode = -1;

					float secondDistance = 99999099;
					int secondNode = -1;

					// Tengo que obtener la información correspondiente a este punto, para ir más rápido.

					for(int node = 0; node < relevantNodes.size(); node++)
					{
						int idxNode = indexOfNode(relevantNodes[node]->nodeId, bd->intPoints);

						assert(idxNode >= 0);

						float distance = 0;
						if(useMVC)
						{
							distance = -BiharmonicDistanceP2P_sorted(weights[idxNode], sortedWeights[idxNode], dp.node->id, bd, relevantNodes[node]->expansion, relevantNodes[node]->precomputedDistances, thresh);
						}
						else
						{
							//distance = -BiharmonicDistanceP2P_sorted(weights[idxNode], sortedWeights[idxNode], dp.node->id, bd, relevantNodes[node]->expansion, relevantNodes[node]->precomputedDistances, thresh);
							distance = -BiharmonicDistanceP2P_HC(weightsClean[idxNode], dp.node->id, bd, relevantNodes[node]->expansion, relevantNodes[node]->precomputedDistances);
						}
						if(bestNode == -1 || bestDistance > distance)
						{
							secondNode = bestNode;
							secondDistance = bestDistance;

							bestNode = node;
							bestDistance = distance;
						}
						else if(secondNode == -1 || secondDistance > distance)
						{
							secondNode = node;
							secondDistance = distance;
						}
					}

					dp.secondInfluences[infl][childIdx] = 1-((float)bestNode/((float)relevantNodes.size()-1));
					continue;

					// Comprovacion que refuerza la hipotesis de que es una parabola con 
					// un unico mínimo en un espacio suficientemente pequeño. 
					
					DefNode* bestNodePtr = relevantNodes[bestNode];
					DefNode* secondNodePtr = relevantNodes[secondNode];

					DefNode *firstNodePtr, *nextNodePtr;
					float firstDistance, nextDistance;

					if(bestNode <= secondNode)
					{
						firstNodePtr = bestNodePtr;
						nextNodePtr = secondNodePtr;
						firstDistance = bestDistance;
						nextDistance = secondDistance;
					}
					else
					{
						nextNodePtr = bestNodePtr;
						firstNodePtr = secondNodePtr;
						nextDistance = bestDistance;
						firstDistance = secondDistance;
					}

					// >> no se porque peta...
					assert(fabs((double)secondNode-bestNode) == 1.0);

					// 3. Obtenemos la derivada en los dos puntos
					float threshold = 0.001;
				
					Point3d dir = nextNodePtr->pos - firstNodePtr->pos;

					float distSegment = dir.Norm();
					float dist = (dir*threshold).Norm();

					std::vector< Point3d > auxPoints; auxPoints.resize(2);
					auxPoints[0] = firstNodePtr->pos - dir*threshold;
					auxPoints[1] = nextNodePtr->pos + dir*threshold;

					//auxPoints[2] = bestNodePtr->pos - dir*threshold;
					//auxPoints[3] = secondNodePtr->pos + dir*threshold;

					// Obtener valores de distancia para estos dos puntos
					vector< vector<double> > weightsTemp; weightsTemp.resize(2);
					vector< vector<int> > weightsSort; weightsSort.resize(2);

					float values[2];
					for(unsigned int i = 0; i< auxPoints.size(); i++)
					{
						mvcAllBindings(auxPoints[i], weightsTemp[i], m->bindings, *m);
						//thresh = 1;
						doubleArrangeElements_wS_fast(weightsTemp[i],weightsSort[i], thresh);
				
						float precompDist = PrecomputeDistancesSingular_sorted(weightsTemp[i], weightsSort[i], bd->BihDistances, thresh);
						values[i] = -BiharmonicDistanceP2P_sorted(weightsTemp[i], weightsSort[i],  dp.node->id, bd, 1.0, precompDist, thresh);
					}

					// Obtener derivadas
					float d1, d2;
				
					d1 = (firstDistance-values[0])/dist;
					d2 = (values[1]-nextDistance)/dist;

					// Calcular punto exacto y guardarlo como parámetro
					float x = (-d1*(distSegment+dist))/(d2-d1);
					//float x = (-d1*(distSegment))/(d2-d1);

					float iniSegment = (firstNodePtr->pos- jt->nodes[0]->pos).Norm();

					// Cae dentro del segmento y hemos averiguado la posición exacta, o bastante aproximada.
					float posRel = x - dist/2.0 + iniSegment;
					float totalLength = (jt->nodes.back()->pos- jt->nodes[0]->pos).Norm();

					int pointIdx = dp.node->id;

					if(d1 > 0 && d2 > 0 )
					{
						// No hay minimo.
						if(bestNode == 0)
							dp.secondInfluences[infl][childIdx] = 1.0;
						else if(bestNode == relevantNodes.size()-1)
							dp.secondInfluences[infl][childIdx] = 0.0;

						continue;
					}
					else if(idInfl == 102 && infl == 0)
						int stop = 0; 

					//TODEBUG: the key point is that it may be only tested with 2 node bone.
					if(posRel <= 0)
					{
						// Se proyecta antes del segmento
						dp.secondInfluences[infl][childIdx] = 1.0;
					}
					else if(posRel >= totalLength)
					{
						// Se proyecta despues del segmento... ver si es 0 lo que toda, quizás depende de la orientación del hueso.
						dp.secondInfluences[infl][childIdx] = 0.0;
					}
					else
					{
						//dp.secondInfluences[infl][childIdx] = 1-(x/distSegment);
						dp.secondInfluences[infl][childIdx] = 1-(posRel/totalLength);
						//dp.secondInfluences[infl][childIdx] = 1-((float)bestNode/((float)relevantNodes.size()-1));

						// Hacer comprobacion para ver que es la menor distancia posible.
						/*vector<double> weightsTemp2;
						Point3d newPoint = bestNodePtr->pos + dir*(x/distSegment); 
						vector<int> weightsSort2; 

						mvcAllBindings(newPoint, weightsTemp2, m->bindings, *m);
						doubleArrangeElements_wS_fast(weightsTemp2,weightsSort2, thresh);
				
						float precompDist = PrecomputeDistancesSingular_sorted(weightsTemp2, weightsSort2, bd->BihDistances, thresh);
						float val = -BiharmonicDistanceP2P_sorted(weightsTemp2, weightsSort2, dp.node->id , bd, 1.0, precompDist, thresh);			
				
						assert(val - bestDistance < threshold && val - secondDistance < threshold);*/
					}

				}
			}
		}
	}

    /*
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
					// Asumo que solo hay un esqueleto... deberian poder vincularse mas.
					joint* jtLinked = grid.bindedSkeletons[0]->getJoint(cell->data->influences[wIdx].label);

					for(int childLinked = 0; childLinked < jtLinked->childs.size(); childLinked++)
					{
						weight wt;
						wt.label = jtLinked->nodeId;
						wt.relativeLabel = jtLinked->childs[childLinked]->nodeId;

						vector<double> unitVector;
						unit(jtLinked->childVector[childLinked], unitVector);

						vector<double> diference;
						vectorDiference( cell->data->embedding, jtLinked->childs[childLinked]->embedding, diference);

						double projection = dotProduct(unitVector, diference);
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
    */
}


 //void Voxelize(scene* snc, Modelo* m, float resolution, float worldScale, bool onlyBorders)
 //{
     /*
	 // Hemos vinculado el grid al modelo y esqueleto cargados.

	 // grid creation for computation
	 grid3d* grid = m->grid = new grid3d();

	 //La resolución y worldScale cogidas de la interficie: parámetros a pasar.
	 grid->res = resolution;
	 grid->worldScale = worldScale;

	 // Hacemos una estimacion de la celda minima para dividir cualquier esqueleto.
	 for(int i = 0; i< snc->skeletons.size(); i++)
	 {
		 float cellSize = grid->cellSize;
		 GetMinSegmentLenght(getMinSkeletonSize((skeleton*)snc->skeletons[i]), cellSize);
		 ((skeleton*)snc->skeletons[i])->minSegmentLength = GetMinSegmentLenght(getMinSkeletonSize((skeleton*)snc->skeletons[i]), cellSize);

		 grid->bindedSkeletons.push_back((skeleton*)snc->skeletons[i]);
	 }

     clock_t begin, end;

     vector< vector<double> >* embedding = NULL;
     if(m->embedding.size() != 0)
     {
         embedding = &(m->embedding);
     }
     else // No se ha cargado ningun embedding.
     {
        printf("No hay cargado ningun embedding. Lo siento, no puedo hacer calculos\n");fflush(0);
         return;
     }

     // Cargamos el grid si ya ha sido calculado
     string sSavingFile = m->sPath + m->sName.c_str()+ "_embedded_grid.dat";
     ifstream ifile(sSavingFile.c_str());
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

         // CONSTRUCCION DEL GRID
         gridInit(*m,*grid);

         if(VERBOSE)
         {
             end=clock();
             cout << double(timelapse(end,begin)) << " s"<< endl;
             begin = end;
             cout << "Etiquetado del grid: ";
         }

         //Tipificamos las celdas según si es interior, exterior, o borde de la maya
         grid->typeCells(*m);

         if(VERBOSE)
         {
             end=clock();
             cout << double(timelapse(end,begin)) << " s"<< endl;
             cout << "Grid cells embedding";
             begin=clock();
         }

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
     */
 //}
 

void renameLinks(vector<int>& links, int group, int newGroup)
{
	for(unsigned int i = 0; i< links.size(); i++)
	{
		if(links[i] == group)
		{
			links[i] = newGroup;
		}
	}
}

int contains(vector<int>& values, int value)
{
	for(unsigned int i = 0; i< values.size(); i++)
	{
		if(values[i] == value)
		{
			return i;
		}
	}
	return -1;
}

void FusionBindings(Modelo& m, vector<vector<int> >& groupBindings)
{	
	/*
	vector<binding*> oldBindings;
	oldBindings.resize(m.bindings.size());
	for(int i = 0; i< m.bindings.size(); i++)
		oldBindings[i] = m.bindings[i];

	m.bindings.clear();
	m.bindings.resize(groupBindings.size());

	for(int bd = 0; bd < groupBindings.size(); bd++)
	{
		m.bindings[bd] = new binding();
		m.bindings[bd]->bindId = bd;
		binding& b = *m.bindings[bd];
		for(int oldBd = 0; oldBd < groupBindings[bd].size(); oldBd++)
		{
			binding& ob = *oldBindings[groupBindings[bd][oldBd]];
			int oldSize = b.pointData.size();
			b.pointData.resize(oldSize + ob.pointData.size());

			for(int pt = 0; pt < ob.pointData.size(); pt++)
			{
				b.pointData[oldSize+pt] = ob.pointData[pt];

				m.modelVertexDataPoint[ob.pointData[pt].modelVert] = oldSize+pt;
				m.modelVertexBind[ob.pointData[pt].modelVert] = bd;

				b.surface.nodes.push_back(new GraphNode(oldSize+pt));
			}

			for(int pt = 0; pt < ob.surface.nodes.size(); pt++)
			{
				for(int con = 0; con < ob.surface.nodes[pt]->connections.size(); con++)
				{
					b.surface.nodes[oldSize+pt]->connections.push_back(b.surface.nodes[oldSize+ob.surface.nodes[pt]->connections[con]->id]);
				}
			}

			for(int tr = 0; tr < ob.virtualTriangles.size(); tr++)
			{
				b.virtualTriangles.push_back(ob.virtualTriangles[tr]);
			}

			b.ntriangles += ob.ntriangles;
		}
	}

	// Guardamos una indirección para tener ordenados los pesos... esto podría variar
	// para optimizar los cálculos.
	int counter = 0;
	m.globalIndirection.resize(m.vn());
	for(int i = 0; i< m.bindings.size(); i++)
	{
		m.bindings[i]->globalIndirection.resize(m.bindings[i]->pointData.size());
		for(int j = 0; j< m.bindings[i]->pointData.size(); j++)
		{
			m.bindings[i]->globalIndirection[j] = counter;
			m.globalIndirection[m.bindings[i]->pointData[j].modelVert] = counter;
			m.bindings[i]->pointData[j].component = i;
			counter++;
		}

		// Acutlizamos también los punteros de los triangulos.
		for(int tr = 0; tr < m.bindings[i]->virtualTriangles.size(); tr++)
		{
			for(int idx = 0; idx < 3; idx++)
			{
				int modelVert = m.bindings[i]->virtualTriangles[tr].Idxs[idx];
				int idBind = m.modelVertexBind[modelVert];
				int idPoint = m.modelVertexDataPoint[modelVert];
				m.bindings[i]->virtualTriangles[tr].pts[idx] = &m.bindings[idBind]->pointData[idPoint];	
			}
		}
	}

	for(int i = 0; i< oldBindings.size(); i++)
	{
		delete oldBindings[i];
		oldBindings[i]=NULL;
	}
	oldBindings.clear();
	*/
}

void AddVirtualTriangles(Modelo& m)
{
	/*
	vector<binding*>& bindings = m.bindings;

	vector<TriangleData>& virtualTriangles = m.virtualTriangles;
	vector< vector<EdgeData> > borderEdges;
	vector< vector<PointData*> > borderPoints;
	vector< vector<int> > borderPointsIdx;

	vector<int> vtBinding;

	printf("hay %d piezas\n", m.bindings.size());

	if(bindings.size() < 2) return;

	// Border Points
	borderPoints.resize(bindings.size());
	borderPointsIdx.resize(bindings.size());
	for(int i = 0; i< bindings.size(); i++ )
	{
		for(int pts = 0; pts < bindings[i]->pointData.size(); pts++)
		{
			if(bindings[i]->pointData[pts].isBorder) 
			{
				borderPoints[i].push_back(&bindings[i]->pointData[pts]);
				borderPointsIdx[i].push_back(pts);
			}
		}

		if(borderPointsIdx[i].size() == 0)
			printf("%d Pieza cerrada.\n", i);
	}

	// Get border edges
	borderEdges.resize(bindings.size());
	for(int i = 0; i< borderPoints.size(); i++ )
	{
		for(int pts = 0; pts < borderPoints[i].size(); pts++)
		{
			int ptId = borderPointsIdx[i][pts];
			for(int con = 0; con < bindings[i]->surface.nodes[ptId]->connections.size(); con++)
			{
				int conId = bindings[i]->surface.nodes[ptId]->connections[con]->id;
				if(bindings[i]->pointData[conId].isBorder)
				{
					EdgeData edg;
					edg.pt1 = borderPoints[i][pts];
					edg.pt2 = &bindings[i]->pointData[conId];
					borderEdges[i].push_back(edg);
					break;
				}
			}
		}
	}

	vector<int> links;
	links.resize(bindings.size());
	for(int i = 0; i< links.size(); i++) links[i]= i;

	//build virtual triangles with borders.
	for(int i = 0; i< borderEdges.size(); i++ )
	{
		for(int edgs = 0; edgs< borderEdges[i].size(); edgs++ )
		{
			TriangleData tr;
			tr.pts[0] = borderEdges[i][edgs].pt1;
			tr.pts[1] = borderEdges[i][edgs].pt2;
			tr.Idxs[0] = borderEdges[i][edgs].pt1->modelVert;
			tr.Idxs[1] = borderEdges[i][edgs].pt2->modelVert;

			double dist = (tr.pts[0]->position-tr.pts[1]->position).Norm()*2;

			double searchValue = 9999999.0;
			PointData* ptFinal = NULL;
			int newBind = -1;
			//for(int j = 0; j < borderPoints.size(); j++)
			for(int j = 0; j < bindings.size(); j++)
			{
				if(i == j) continue;
				
				//for(int pts = 0; pts < borderPoints[j].size(); pts++)
				for(int pts = 0; pts < bindings[j]->pointData.size(); pts++)
				{
					//double auxDist = (tr.pt1->position-borderPoints[j][pts]->position).Norm() + (tr.pt2->position-borderPoints[j][pts]->position).Norm();
					double l1 = (tr.pts[0]->position - bindings[j]->pointData[pts].position).Norm();
					double l2 = (tr.pts[1]->position - bindings[j]->pointData[pts].position).Norm();
					double auxDist = l1+l2;
					if(auxDist < searchValue)
					{
						searchValue = auxDist;
						ptFinal = &bindings[j]->pointData[pts];
						//ptFinal = borderPoints[j][pts];
						newBind = j;
					}
				}
			}
			
			if(ptFinal)
			{
				tr.pts[2] = ptFinal;
				tr.Idxs[2] = ptFinal->modelVert;
				virtualTriangles.push_back(tr);

				vtBinding.push_back(ptFinal->modelVert);

				if(links[newBind] != links[i])
					renameLinks(links, newBind, links[i]);

				//printf("Hemos unido %d con %d\n", i ,newBind);
			}
		}
	}

	int groupCounter = 0;
	vector<int> groupIds;
	vector< vector<int> > groupBindings;
	for(int i = 0; i< links.size(); i++)
	{
		int id = contains(groupIds, links[i]);
		if(id < 0)
		{
			groupIds.push_back(links[i]);
			groupBindings.resize(groupBindings.size()+1);
			groupBindings.back().push_back(i);
			groupCounter++;
		}
		else
		{
			groupBindings[id].push_back(i);
		}
	}

	printf("Quedan %d grupos\n", groupIds.size());

	for(int i = 0; i < vtBinding.size(); i++)
	{
		vtBinding[i] = m.modelVertexBind[vtBinding[i]];
		m.bindings[vtBinding[i]]->virtualTriangles.push_back(m.virtualTriangles[i]);
	} 

	FusionBindings(m, groupBindings);

	for(int grp = 0; grp < m.bindings.size(); grp++)
	{
		printf("Grupo %d: %d elementos\n", grp, m.bindings[grp]->pointData.size());
	}
	fflush(0);
	*/
	/*
	for(int i = 0; i< bindings.size(); i++ )
	{
		if(borderEdges[i].size() == 0)
		{
			// vamos a unir de momento por el punto mas cercano.
			printf("%d Pieza cerrada...\n", i);
			int closestBind = -1;
			int closestPoint1 = -1;
			int closestPoint2 = -1;
			double dist = 99999;
			for(int j = 0; j< bindings.size(); j++)
			{
				if(j==i) continue;

				for(int pt1 = 0; pt1 < bindings[i]->pointData.size(); pt1++)
				{
					for(int pt2 = 0; pt2 < bindings[j]->pointData.size(); pt2++)
					{
						double newDist = (bindings[j]->pointData[pt1].position - bindings[j]->pointData[pt2].position).Norm();
						if(newDist< dist)
						{
							closestBind = j;
							closestPoint1 = pt1;
							closestPoint2 = pt2;
							dist = newDist;
						}
					}
				}

				TriangleData tr;
				tr.pt1 = &bindings[i]->pointData[closestPoint1];
				tr.pt2 = &bindings[i]->pointData[closestPoint2];
				// DANGER! TODEBUG: Esto podria petar porque no hago ninguna comprobacion
				tr.pt3 = &bindings[i]->pointData[bindings[j]->surface.nodes[closestPoint1]->connections[0]->id];
			}
		}
	}
	*/

	//printf("Hemos creado %d triangulos\n", virtualTriangles.size());
}

void normalizeDistances(Modelo& m)
{
	vector<binding*>& bindings = m.bindings;

	// Normalizacion de las matrices A para calcular con todas a la vez
	// Normalizamos segun las distancias del mundo real.
	if(bindings.size() > 1)
	{
		for(int i = 0; i< bindings.size(); i++)
		{
			// Este caso es muy raro, en principio deberia haber al menos un triangulo.
			if(bindings[i]->pointData.size() < 2)
				continue;

			// buscare la distancia mas grande para normalizar, asi conseguimos que sea mas estable.
			// Aunque no tengo claro si es mejor normalizar.
			double maxDistance = 0;
			int maxX = 0, maxY = 0;
			for(int ptX = 0; ptX < bindings[i]->surface.nodes.size(); ptX++)
			{
				int idX = bindings[i]->surface.nodes[ptX]->id;
				for(int ptY = 0; ptY < bindings[i]->surface.nodes[ptX]->connections.size(); ptY++)
				{
					int idY = bindings[i]->surface.nodes[ptX]->connections[ptY]->id;
					double dist = bindings[i]->BihDistances.get(idX,idY);
					if(dist > maxDistance)
					{
						maxDistance = dist;
						maxX = idX;
						maxY = idY;
					}

					double distance1 = (bindings[i]->pointData[idX].node->position - bindings[i]->pointData[idY].node->position).Norm();
					double ratio1 = distance1/bindings[i]->BihDistances.get(idX,idY);
					int stop = 0;
				}
			}

			/*
			for(int x = 0; x < bindings[i]->BihDistances.size; x++)
			{
				for(int y = x+1; y < bindings[i]->BihDistances.size; y++)
				{
					double dist = bindings[i]->BihDistances.get(x,y);
					if(dist > maxDistance)
					{
						maxDistance = dist;
						maxX = x;
						maxY = y;

						double distance1 = (bindings[i]->pointData[maxX].position - bindings[i]->pointData[maxY].position).Norm();
						double ratio1 = distance1/bindings[i]->BihDistances.get(maxX,maxY);
						int stop = 0;
					}

				}
			}
			*/

			double distance = (bindings[i]->pointData[maxX].node->position - bindings[i]->pointData[maxY].node->position).Norm();
			
			if(maxX == maxY) // la diagonal es 0 -> no deberia dar.
				continue;

			double ratio = distance/bindings[i]->BihDistances.get(maxX,maxY);
			
			if(distance == 0) 
				continue;

			printf("\n\nNormalization ratio:%f\n\n", ratio);fflush(0);
			// Normalización
			for(int x = 0; x < bindings[i]->BihDistances.size; x++)
			{
				for(int y = x+1; y < bindings[i]->BihDistances.size; y++)
				{
					bindings[i]->BihDistances.set(x,y, bindings[i]->BihDistances.get(x,y)*ratio);
				}
			}
		}
	}
}

bool ComputeEmbeddingWithBD(Modelo& model, bool withPatches)
{
	vector<binding*>& bds = model.bindings;

	for(int bind = 0; bind < bds.size(); bind++)
	{	
		// Computation indices
		vector<int> indices;
		indices.resize(bds[bind]->pointData.size());
		for(int idx = 0; idx < bds[bind]->pointData.size(); idx++)
		{
			//indices.push_back(idx);
			//indices.push_back(bds[bind]->pointData[idx].node->id);
			indices[idx] = bds[bind]->pointData[idx].node->id;
		}

		printf("\n\nCalculo de A para embeding %d: [%d puntos]\n", bind, bds[bind]->pointData.size());
		fflush(0);
		bindingBD( model, bds[bind], indices, bds[bind]->BihDistances, withPatches);
	}
	return true;
}

bool LoadEmbeddings(Modelo& m, char* bindingFileName)
{

	ifstream finbin;
	finbin.open(bindingFileName, ios::in |ios::binary);
	printf("Cargando embedding en fichero de texto: %s\n", bindingFileName);

	bool success = finbin.is_open();
	if(!success) return false;

	int bindSize = 0;
	finbin.read( (char*) &bindSize,  sizeof(int) );

	// Debe corresponder el numero de bindings
	if(bindSize != m.bindings.size())
		return false;

	for(int bind = 0; bind < m.bindings.size(); bind++)
	{
		int pointsSize = 0;
		finbin.read( (char*) &pointsSize,  sizeof(int) );

		// Debe corresponder el numero de puntos
		if(pointsSize != m.bindings[bind]->surface.nodes.size())
			return false;

		m.bindings[bind]->BihDistances.resize(pointsSize);
		for(int row = 0; row < m.bindings[bind]->BihDistances.size; row++)
		{
			for(int col = row; col < m.bindings[bind]->BihDistances.size; col++)
			{
				double value = 0;
				finbin.read( (char*) &value,  sizeof(double) );
				m.bindings[bind]->BihDistances.set(row,col,value);
			}
		}
	}
	
	finbin.close();
	return true;
}

void SaveEmbeddings(Modelo& model, char* fileName, bool ascii)
{
	char asciiFile[100];
	sprintf(asciiFile, "%s/bind_%s.txt", model.sPath.c_str(), model.sName.c_str());

	char binFile[100];
	sprintf(binFile, "%s/bind_%s.bin", model.sPath.c_str(), model.sName.c_str());

	FILE* foutascii = NULL;
	ofstream foutbin;
	if(ascii)
	{
        foutascii = fopen(asciiFile, "w");
		printf("Guardado embedding en fichero de texto: %s\n", asciiFile);
	}
	else
	{
		foutbin.open(binFile, ios::binary);
		printf("Guardado embedding en fichero de texto: %s\n", binFile);
	}

	if(ascii)
		fprintf(foutascii,"%d\n", model.bindings.size());
	else
	{
		int size = model.bindings.size();
		foutbin.write((const char*) &size, sizeof(int));
	}
	for(int bind = 0; bind < model.bindings.size(); bind++)
	{
		if(ascii)
			fprintf(foutascii,"%d\n", model.bindings[bind]->pointData.size());
		else
		{
			int ss = model.bindings[bind]->pointData.size();
			foutbin.write((const char*) &ss, sizeof(int));
		}

		for(int row = 0; row < model.bindings[bind]->BihDistances.size; row++)
			for(int col = row; col < model.bindings[bind]->BihDistances.size; col++)
			{
				if(ascii)
					fprintf(foutascii,"%f\n", model.bindings[bind]->BihDistances.get(row,col));
				else
				{
					double val = model.bindings[bind]->BihDistances.get(row,col);
					foutbin.write((const char*) &val, sizeof(double));
				}
			}
	}

	if(ascii)
		fclose(foutascii);
	else
		foutbin.close();
	
}