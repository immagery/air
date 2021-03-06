#include "AirSegmentation.h"

//#include <Computation\Segmentation.h>
#include <Computation\mvc_interiorDistances.h>
#include <Computation/BiharmonicDistances.h>

#include <DataStructures\InteriorDistancesData.h>

#include <DataStructures/Scene.h>

//#include <cuda/cudaMgr.cuh>

#include <omp.h>

#include <algorithm>

#define LOCAL_COMP_DEBUG true

// Esta funci�n deber�a actualizar los datos y relanzar los c�lculos necesarios.
bool AirRig::updateDefGroups()
{
	map<int, bool> updated;
	for(int gr = 0; defRig.defGroups.size(); gr++)
	{
		if(defRig.defGroups[gr]->dirtyFlag)
		{
			for(int defNodeIdx = 0; defNodeIdx < defRig.defGroups[gr]->deformers.size(); defNodeIdx++)
				defRig.defGroups[gr]->deformers[defNodeIdx].dirtyFlag = true;

			if(!defRig.defGroups[gr]->transformation)
			{
				// Este caso no deber�a darse porque las estructuras deber�an estar mantenidas correctamente.
			}

			// Volver a proponer los nodos, sin perder informaci�n de usuario.
		}

		// Marcamos el joint como hecho, as� podemos llevar un control de la coherencia de los datos.
		updated[defRig.defGroups[gr]->transformation->nodeId] = true;

	}

	return true;
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

int getSignificantWeights(vector<ordWeight>& weights,  VectorXf& cuttedWeights, VectorXi& cuttedIndexes)
{
	ordIdx orderByIdx;
    ordWeight orderByWeight;

	//1. Ordenar el mapa de pesos.
	std::sort(weights.begin(), weights.end(), orderByWeight);

	//2. Buscar punto de corte.
	float negSum = 0;
	float posSum = 0;
	int thresholdCount = 0;
	vector<ordWeight>::iterator thIt;
	for(thIt = weights.end(); thIt != weights.begin();)
	{
		thIt--;
		float w = thIt->weight;
		if(w < 0)
		{
			negSum += w;
			thresholdCount++;
		}
		else
		{
			posSum += w;
			thresholdCount++;
			if(posSum > -negSum) break;
		}
	}

	if(weights.begin() == thIt)
	{
		// Deberiamos cortar todo... es un poco animal
	}
	else
	{
		//3. Ordenar el mapa de pesos.
		std::sort(weights.begin(), thIt, orderByIdx);
	}

	// 5. Construccion de los vectores resultado
	int newSize = weights.size() - thresholdCount;
	cuttedWeights.resize(newSize);
	cuttedIndexes.resize(newSize);
	vector<ordWeight>::iterator it = weights.begin();
	for(int i = 0; i< newSize; i++, it++)
	{
		cuttedWeights[i] = it->weight;
		cuttedIndexes[i] = it->idx;
	}
	
	return newSize;
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

double PrecomputeDistancesSingular_sorted(vector<double>& weights, vector<int>& indirection, symMatrixLight& BihDistances, double threshold)
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

void updateAirSkinning(DefGraph& graph, Modelo& model)
{
	clock_t ini = clock();
	binding* bd = model.bind;

	long long mvcTime = 0;
	long long arrangementTime = 0;
	long long precompTime = 0;

	int defCount = 0;

	FILE* fout = fopen("C:\\Users\\chus\\Documents\\dev\\Data\\models\\modelo001.txt", "w");

	// Updating deformer's info
	for(unsigned int defId = 0; defId< graph.deformers.size(); defId++)
    {
		// Only the dirty deformers
		if(graph.deformers[defId]->dirtyFlag)
		{
			clock_t ini1 = clock();
			// Mean value coordinates computation.
			mvcAllBindings(graph.deformers[defId]->pos, 
						   graph.deformers[defId]->MVCWeights,
						   model);
			clock_t fin1 = clock();

			mvcTime += fin1-ini1;
			
			// TOREMOVE: it's only for select adaptative threshold
			graph.deformers[defId]->cuttingThreshold = 1;

			clock_t ini2 = clock();
			// TO_OPTIMIZE: Weights sort, now with bubble approach
			doubleArrangeElements_wS_fast(graph.deformers[defId]->MVCWeights, 
										  graph.deformers[defId]->weightsSort, 
										  graph.deformers[defId]->cuttingThreshold);
			
			clock_t fin2 = clock();
			arrangementTime += fin2-ini2;

			//TODEBUG: BihDistances as a matrix
			// By now just one embedding
			clock_t ini3 = clock();
			graph.deformers[defId]->precomputedDistances = 
							PrecomputeDistancesSingular_sorted(
							graph.deformers[defId]->MVCWeights, 
							graph.deformers[defId]->weightsSort, 
							bd->BihDistances[0], 
							graph.deformers[defId]->cuttingThreshold);

			clock_t fin3 = clock();
			precompTime += fin3-ini3;

			graph.deformers[defId]->dirtyFlag = false;
			graph.deformers[defId]->segmentationDirtyFlag = true;

			defCount++;

			fprintf(fout, "[%d] -> %f\n", defId, graph.deformers[defId]->precomputedDistances);
		}
	}

	fclose(fout);
	
	printf("Media en MVC: %f\n", (double)mvcTime/CLOCKS_PER_SEC/defCount);
	printf("Media en sorting: %f\n", (double)arrangementTime/CLOCKS_PER_SEC/defCount);
	printf("Media en precomputing: %f\n", (double)precompTime/CLOCKS_PER_SEC/defCount);

	// By now we are reducing the process to just one binding.

	// Updating Segmentation
	clock_t ini4 = clock();
	segmentModelFromDeformers(model, model.bind, graph);
	clock_t fin4 = clock();

	FILE* fout3 = fopen("C:\\Users\\chus\\Documents\\dev\\Data\\models\\segmentations\\model_segmentation.txt", "w");
	for(int j = 0; j< model.bind->pointData.size(); j++)
	{
		fprintf(fout3, "%d %d\n", j, model.bind->pointData[j].segmentId);
	}
	fclose(fout3);

	// Update Smooth propagation
	clock_t ini5 = clock();
	propagateHierarchicalSkinningOpt(model, model.bind, graph, 0);
	clock_t fin5 = clock();

	clock_t ini6 = clock();
	// Compute Secondary weights ... by now compute all the sec. weights
	computeSecondaryWeights(model, model.bind, graph);
	clock_t fin6 = clock();
	clock_t fin = clock();

	printf("\n\nA. Segmentar modelo: %f\n", timelapse(ini4,fin4));
	printf("B. Propagar pesos: %f\n", timelapse(ini5,fin5));
	printf("C. Calculo de pesos secundarios: %f\n", timelapse(ini6,fin6));
	printf("D. Calculo de pesos total: %f ms\n\n\n", timelapse(ini,fin)); fflush(0);
	
}


// Temp. deber�a estar unificado... porque hay c�digo repetido.
/*
void getCompactRepresentationSegm(cudaModel& model, Modelo& m)
{
	// Load constants
	model.npts = m.nodes.size();
	model.ntri = m.triangles.size();

	model.hostPositions = (PRECISION*) malloc(model.npts*sizeof(PRECISION)*3);
	model.hostTriangles = (int*) malloc(model.ntri*sizeof(int)*3);

	// Copiamos los vertices
	for(int i = 0; i< m.nodes.size(); i++)
	{
		model.hostPositions[i*3+0] = m.nodes[i]->position.x();
		model.hostPositions[i*3+1] = m.nodes[i]->position.y();
		model.hostPositions[i*3+2] = m.nodes[i]->position.z();
	}

	// Copiamos los triangulos
	for(int i = 0; i< m.triangles.size(); i++)
	{
		model.hostTriangles[i*3+0] = m.triangles[i]->verts[0]->id;
		model.hostTriangles[i*3+1] = m.triangles[i]->verts[1]->id;
		model.hostTriangles[i*3+2] = m.triangles[i]->verts[2]->id;
	}


	// Copiamos la matriz de distancias.
	model.hostBHDistances = (float*) malloc((model.npts*model.npts+model.npts)/2*sizeof(float));
	int count = 0; 
	for(int i = 0; i< m.bind->BihDistances[0].size; i++)
	{
		for(int j = 0; j<= i; j++)
		{
			model.hostBHDistances[count] = m.bind->BihDistances[0].get(i,j);
			count++;
		}
	}

}

*/

void updateAirSkinningWithCuda(DefGraph& graph, Modelo& model)
{
	/*

	// Test with MVC
	//---------------
	// Inicio init CUDA
	//----------------

	Modelo* m = &model; 
	
	vector<cudaModel> models(1);

	getCompactRepresentationSegm(models[0], *m);

	cudaManager cudaMgr;
	cudaMgr.loadModels(models.data(), 1);

	//---------------
	// Fin init CUDA
	//----------------

	clock_t ini = clock();
	binding* bd = model.bind;

	// Updating deformer's info
	for(unsigned int defId = 0; defId< graph.deformers.size(); defId++)
    {
		// Only the dirty deformers
		if(graph.deformers[defId]->dirtyFlag)
		{
			// Mean value coordinates computation.
			PRECISION3 point;
			point.x = graph.deformers[defId]->pos.x();
			point.y = graph.deformers[defId]->pos.y();
			point.z = graph.deformers[defId]->pos.z();
			//cudaMgr.cudaMVC(point, graph.deformers[defId]->MVCWeights.data(), 0, 0);
			
			// TOREMOVE: it's only for select adaptative threshold
			graph.deformers[defId]->cuttingThreshold = 1;

			// TOOPTIMIZE: Weights sort, now with bubble approach
			doubleArrangeElements_wS_fast(graph.deformers[defId]->MVCWeights, 
										  graph.deformers[defId]->weightsSort, 
										  graph.deformers[defId]->cuttingThreshold);

			//TODEBUG: BihDistances as a matrix
			// By now just one embedding
			graph.deformers[defId]->precomputedDistances = PrecomputeDistancesSingular_sorted(graph.deformers[defId]->MVCWeights, 
																							  graph.deformers[defId]->weightsSort, 
																							  bd->BihDistances[0], 
																							  graph.deformers[defId]->cuttingThreshold);
			graph.deformers[defId]->dirtyFlag = false;
			graph.deformers[defId]->segmentationDirtyFlag = true;
		}
	}

	// Test with MVC
	cudaMgr.freeModels();

	// By now we are reducing the process to just one binding.

	// Updating Segmentation
	segmentModelFromDeformers(model, model.bind, graph);

	// Update Smooth propagation
	propagateHierarchicalSkinning(model, model.bind, graph);

	// Compute Secondary weights ... by now compute all the sec. weights
	computeSecondaryWeights(model, model.bind, graph);

	clock_t fin = clock();
	printf("Calculo de pesos total: %f ms\n", timelapse(fin,ini)*1000); fflush(0);

	*/
	
}

void createTraductionTableOpt(DefGroup* group, vector<int>& traductionTable, int idNode, bool onlyRoot)
{
	// Los nodos correspondientes al hueso jt
	for(unsigned int i = 0; i< group->deformers.size(); i++)
    {
		int node = group->deformers[i].nodeId % CROP_NODE_ID;
        traductionTable[node] = idNode;
    }

	// Si solo queremos anadir la base nos quedamos aqui.
	if(onlyRoot) return;

    // Descendemos por la jerarqu�a marcando los nodos de los huesos hijos.
    for(unsigned int jtChild = 0; jtChild < group->relatedGroups.size(); jtChild++)
    {
		createTraductionTableOpt(group->relatedGroups[jtChild], traductionTable, idNode);
    }
}

void createTraductionTable(DefGroup* group, map<int, int>& traductionTable, int idNode, bool onlyRoot)
{
	    // Los nodos correspondientes al hueso jt
	for(unsigned int i = 0; i< group->deformers.size(); i++)
    {
		int node = group->deformers[i].nodeId;
        traductionTable[group->deformers[i].nodeId] = idNode;
    }

	// Si solo queremos anadir la base nos quedamos aqui.
	if(onlyRoot) return;

    // Descendemos por la jerarqu�a marcando los nodos de los huesos hijos.
    for(unsigned int jtChild = 0; jtChild < group->relatedGroups.size(); jtChild++)
    {
		createTraductionTable(group->relatedGroups[jtChild], traductionTable, idNode);
    }
}

#include <chrono>
using namespace std::chrono;

void initSurfaceWeightsSmoothingOpt(Modelo& m, 
									binding* bd, 
									int nodeId, 
									vector<float>& weights,
									vector<float>& weightsAux,
									vector<unsigned int>& indicesToCompute,
									int& lastIndex,
									int surfIdx)
{

	//auto ini00 = high_resolution_clock::now();
	lastIndex = 0;
	int totalElems = 0;
	int totalPosElems = 0;

	int length = bd->surfaces[surfIdx].nodes.size();

	// recorrer todo el modelo e inicializar los datos para la transmision
	std::fill(weights.begin(), weights.end(), 0.0);
	std::fill(weightsAux.begin(), weightsAux.end(), 0.0);
	for(int idx = 0; idx < length; idx++ )
	{	
		PointData& pd = bd->pointData[bd->surfaces[surfIdx].nodes[idx]->id];
		pd.node->visited = false;
		if(pd.ownerLabel == nodeId)
		{
			weights[idx] = 1.0;
			weightsAux[idx] = 1.0;
		}
	}
	//auto fin00 = high_resolution_clock::now();

	//auto ini01 = high_resolution_clock::now();
	// Seleccionamos para procesar solo lo necesario.
	for(int idx = 0; idx < length; idx++ )
	{
		float value = weights[idx]; 
		PointData& pd = bd->pointData[bd->surfaces[surfIdx].nodes[idx]->id];

		bool difValues = false;
		for(int n=0; !difValues && n<pd.node->connections.size(); n++)
		{
			int neighId = pd.node->connections[n]->pieceId;
			difValues |= (value != weights[neighId]);
		}

		if(difValues)
		{
			pd.node->visited = true;
			indicesToCompute[lastIndex] = pd.node->id;
			lastIndex++;
		}
	}
}

void initSurfaceWeightsSmoothing(Modelo& m, binding* bd, vector< int >& front, int nodeId)
{
	// recorrer todo el modelo e inicializar los datos para la transmision
	front.clear();
	for(int idx = 0; idx < bd->pointData.size(); idx++ )
	{	
		if(bd->pointData[idx].ownerLabel != nodeId)
		{
			bd->pointData[idx].node->visited = false;
			bd->pointData[idx].itPass = 0;
			bd->pointData[idx].ownerWeight = 0.0;
			bd->pointData[idx].tempOwnerWeight = 0.0;
		}
		else
		{
			bd->pointData[idx].node->visited = true;
			bd->pointData[idx].itPass = 0;
			bd->pointData[idx].ownerWeight = 1.0;
			bd->pointData[idx].tempOwnerWeight = 1.0;
			front.push_back(idx);
		}
	}
}

// Propaga el peso a lo largo de la superficie, teniendo en cuenta una segmentaci�n.
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

void SmoothFromSegment(Modelo& m, binding* bd, DefGroup* group, int frontId)
{
    // Fronts creation using frontId
    vector< int > front(0);

    // Init model for weights smoothing
	if(VERBOSE) printf("\n-- initCellWeightsSmoothing --\n");fflush(0);
    initSurfaceWeightsSmoothing(m, bd, front, frontId);

    // Smoothing (sin corte jer�rquico)
	if(VERBOSE) printf("-- Smoothing -- \n");fflush(0);

	// get the global or local smoothing value
	int smoothingPasses = group->smoothingPasses;
	float realSmooth = group->smoothPropagationRatio;

    weightsSmoothing(m, bd, front, realSmooth, frontId, smoothingPasses);
	//weightsNoSmooting(m, bd, front, frontId);
}

/* [ SmoothFromSegmentOpt ] (no header)

DESCRIPTION:
	Expands the weights of a branch of the skeleton, this is a part of the hierarchical
	smoothing of weights after a proper segmentation.

PARAMS:
	m: model
	bd: binding for this model
	group: deformation group to expand
	frontId: the node id of the root of this branch of the skeleton
	weightsT1: temporal data structure for compute smooth weights eficiently.
	weightsT2: temporal data structure for compute smooth weights eficiently.
	indicesToCompute: temporal data structure for compute smooth weights eficiently.
*/

void SmoothFromSegmentOpt(Modelo& m, binding* bd, DefGroup* group, int frontId, 
						  vector<float>& weightsT1, vector<float>& weightsT2, 
						  vector<unsigned int>& indicesToCompute, int surfIdx)
{
	int lastIndex = 0;

    // Init model for weights smoothing
    initSurfaceWeightsSmoothingOpt(m, bd, frontId, weightsT1, weightsT2, indicesToCompute, lastIndex, surfIdx);

	// get the global or local smoothing value
	int smoothingPasses = group->smoothingPasses;
	float realSmooth = group->smoothPropagationRatio;

    weightsSmoothing_opt(m, bd, realSmooth, 
						frontId, smoothingPasses, weightsT1, 
						weightsT2, indicesToCompute, lastIndex, surfIdx);

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

void initData(Modelo& m, binding* bd,
                      vector< int >& front,
                      float smoothPropagationRatio,
                      int idFront,
					  int smoothingPasses,
					  vector<float>& weightsT1,
					  vector<float>& weightsT2,
					  vector<bool>& visited,
					  vector< vector<int> >& neighbours,
					  vector<unsigned int>& indicesToCompute,
					  int& lastIndex)
{

	int numOfPoints = bd->pointData.size();	

	// Temporal data to process
	lastIndex = 0;

	vector<float>* ptWT1 = &weightsT2;
	vector<float>* ptWT2 = &weightsT1;

	// Inicializamos los pesos necesarios.
	for(unsigned int ptIdx = 0; ptIdx < numOfPoints; ptIdx++)
    {
		PointData& pd = bd->pointData[ptIdx];
		visited[ptIdx] = true;
		neighbours[ptIdx].resize(pd.node->connections.size());

		for(int ni = 0; ni < neighbours[ptIdx].size(); ni++)
			neighbours[ptIdx][ni] = pd.node->connections[ni]->id;

		if(pd.node->visited)
		{
			indicesToCompute[lastIndex] = ptIdx;
			(*ptWT2)[ptIdx] = pd.ownerWeight;

			for(int n = 0; n< pd.node->connections.size(); n++)
			{
				if(!pd.node->connections[n]->visited)
				{
					pd.node->connections[n]->visited = true;
					lastIndex++;
				}
			}
		}
	}
}


// Propaga el peso a lo largo de la superficie, teniendo en cuenta una segmentaci�n.
void weightsSmoothing_opt(Modelo& m, binding* bd,
						  float smoothPropagationRatio,
						  int idFront,
						  int smoothingPasses,
						  vector<float>& ptWT1_ini,
						  vector<float>& ptWT2_ini,
						  vector<unsigned int>& indicesToCompute,
						  int& lastIndex,
						  int surfIdx)

{
    int iter = 0;
	int distance = 1;

	//FixedValue only for test... seems that there is no effects while the value is enough big.
	// I think that this is because it scales all the values, maybe I can use the biggest one
	// for preserving the precision. I can do it at the beginning with the biggest edge length.
	// By now, only for test i will use a big value;

	vector<float>* ptWT1 = &ptWT1_ini;
	vector<float>* ptWT2 = &ptWT2_ini;

	smoothPropagationRatio = 100;
	
	int threads = omp_get_max_threads();

	while(iter < smoothingPasses)
    {		
		int tempLastIndex = lastIndex;

		//int numElemems = (tempLastIndex/threads);
		//#pragma omp for schedule(static, numElemems)

		int numBlocks = 8;
		int elementsPerBlock = (int)ceil((float)tempLastIndex / (float)numBlocks);

		#pragma omp parallel for
		for (int blockIdx = 0; blockIdx < numBlocks; blockIdx++)
		{
			int iniBlock = blockIdx * elementsPerBlock;
			int finBlock = iniBlock + elementsPerBlock;

			//for (int ptIdx = 0; ptIdx < tempLastIndex; ptIdx++)
			for (int ptIdx = iniBlock; ptIdx < finBlock && ptIdx < tempLastIndex; ptIdx++)
			{
				int ownIdx = indicesToCompute[ptIdx];
				PointData& pd = bd->pointData[ownIdx];

				int localPdIdx = pd.node->pieceId;

				float value = 0;

				// Get the sum of values
				int cn = pd.node->connections.size();
				// Para cada elemento del frente realizamos la expansi�n.
				for (unsigned int neighboursIdx = 0; neighboursIdx < cn; neighboursIdx++)
				{
					int localNeighbourId = pd.node->connections[neighboursIdx]->pieceId;
					int neighbourId = pd.node->connections[neighboursIdx]->id;
					value += (*ptWT1)[localNeighbourId];

					#pragma omp critical
					{
						if (!pd.node->connections[neighboursIdx]->visited)
						{
							pd.node->connections[neighboursIdx]->visited = true;
							indicesToCompute[lastIndex] = neighbourId;
							lastIndex++;
						}
					}
				}

				// Do the mean
				if (cn > 0) value /= cn;

				(*ptWT2)[localPdIdx] = value;
			}

		}

		// Change of array to compute weights
		vector<float>* tempVectPtr = ptWT1;
		ptWT1 = ptWT2;
		ptWT2 = tempVectPtr;
        iter++;
    }

	int numElemems2 = (bd->surfaces[surfIdx].nodes.size()/threads);
	#pragma omp for schedule(static, numElemems2)
	for(int ptIdx = 0; ptIdx < bd->surfaces[surfIdx].nodes.size(); ptIdx++)
    {
		PointData& pd = bd->pointData[bd->surfaces[surfIdx].nodes[ptIdx]->id];

		if((*ptWT1)[ptIdx] > 0)
		{
			int i = findWeight(pd.auxInfluences, idFront);
			if(i < 0)
				pd.auxInfluences.push_back(weight(idFront, (*ptWT1)[ptIdx]));
			else
				pd.auxInfluences[i] = weight(idFront, (*ptWT1)[ptIdx]);
		}
	}
}

// Propaga el peso a lo largo de la superficie, teniendo en cuenta una segmentaci�n.
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

	float weightsSum = 0;
	float weightsCount = 0;
    if(VERBOSE)printf("Front size: %d\n", front.size());
	int frontSize = 0;
	while(iter < smoothingPasses)
    {
		/*vector<int> toAddToFront;
        for(unsigned int frIdx = 0; frIdx < front.size(); frIdx++)
        {
            int frontIdx = front[frIdx];
			PointData& pd = bd->pointData[frontIdx];
			GraphNode* sn = pd.node;//bd->mainSurface->nodes[frontIdx];

			weightsSum = 0;
			weightsCount = 0;
            // Para cada elemento del frente realizamos la expansi�n.
			for(unsigned int neighboursIdx = 0; neighboursIdx < sn->connections.size(); neighboursIdx++)
            {
				GraphNode* snNeighbour =  sn->connections[neighboursIdx];
				bool visitedEarly = true;
				if(!snNeighbour->visited)
				{
					snNeighbour->visited = true;
					toAddToFront.push_back(snNeighbour->id);
					//bd->pointData[snNeighbour->id].ownerWeight = 0.0;
					//bd->pointData[snNeighbour->id].tempOwnerWeight = 0.0;
					visitedEarly = false;
				}

				// Distancia entre puntos... cuando tenga biharmonic distances ya estar� calculado.
				Vector3d vec = pd.node->position - bd->pointData[snNeighbour->id].node->position;
				float edgeDistance = vec.norm();
				float distanceWeight = computeWeightProportional(edgeDistance, smoothPropagationRatio, true);

				// La media es ponderada
				weightsSum += bd->pointData[snNeighbour->id].ownerWeight;//*distanceWeight;

				weightsCount += 1;//*distanceWeight;
			}

			if(weightsCount > 0)
				weightsSum = weightsSum / weightsCount;
			else
			{
				printf("Como puede ser que no haya ningun peso?.\n");
				weightsSum = 0;
			}

			pd.tempOwnerWeight = weightsSum;


        }

		for(unsigned int frIdx = 0; frIdx < front.size(); frIdx++)
        {
            PointData& pd = bd->pointData[front[frIdx]];
			pd.ownerWeight = pd.tempOwnerWeight;
			pd.tempOwnerWeight = 0;
		}

		for(unsigned int frIdx = 0; frIdx < toAddToFront.size(); frIdx++)
		{
			front.push_back(toAddToFront[frIdx]);
		}
		*/


		for(unsigned int ptIdx = 0; ptIdx < bd->pointData.size(); ptIdx++)
        {
			PointData& pd = bd->pointData[ptIdx];
			GraphNode* sn = pd.node;

			//if(!sn->visited)
			{
				weightsSum = 0;
				weightsCount = 0;
				// Para cada elemento del frente realizamos la expansi�n.
				for(unsigned int neighboursIdx = 0; neighboursIdx < sn->connections.size(); neighboursIdx++)
				{
					GraphNode* snNeighbour =  sn->connections[neighboursIdx];

					//Vector3d vec = pd.node->position - bd->pointData[snNeighbour->id].node->position;
					//float edgeDistance = vec.norm();
					//float distanceWeight = computeWeightProportional(edgeDistance, smoothPropagationRatio, true);

					// La media es ponderada
					weightsSum += bd->pointData[snNeighbour->id].ownerWeight;
					weightsCount += 1;
				}

				if(weightsCount > 0)
					weightsSum = weightsSum / weightsCount;
				else
				{
					printf("[weightsSmoothing] Como puede ser que no haya ningun peso?.\n");
					weightsSum = 0;
				}
				pd.tempOwnerWeight = weightsSum;
			}

        }

		for(unsigned int ptIdx = 0; ptIdx < bd->pointData.size(); ptIdx++)
        {
            PointData& pd = bd->pointData[ptIdx];
			//if(!pd.node->visited)
			{
				pd.ownerWeight = pd.tempOwnerWeight;
				pd.tempOwnerWeight = 0;
			}
		}

        iter++;
    }

	for(unsigned int ptIdx = 0; ptIdx < bd->pointData.size(); ptIdx++)
    {
		PointData& pd = bd->pointData[ptIdx];

		int i = findWeight(pd.auxInfluences, idFront);
		if(i < 0)
			pd.auxInfluences.push_back(weight(idFront, pd.ownerWeight));
		else
			pd.auxInfluences[i] = weight(idFront, pd.ownerWeight);
	}
}

void traducePartialSegmentation(Modelo& m, binding* bd, map<int, int>& traductionTable, int surfIdx)
{
    // Recorrer cada vertice de la maya con un iterador y clasificar los vertices.
	for(int VertIdx = 0; VertIdx < bd->surfaces[surfIdx].nodes.size(); VertIdx++ )
	{
        // Assegurar que los indices guardados en los vertices de la maya estan bien.
        assert(VertIdx >= 0 && VertIdx < m.vn());
        PointData& pd = bd->pointData[bd->surfaces[surfIdx].nodes[VertIdx]->id];

        if(pd.segmentId>=0)
        {
            pd.ownerLabel = traductionTable[pd.segmentId];
        }
        else
        {
			// This is common when the segmentation is not possible for each part
            //printf("[ver traduceParcialSegmentation(...)]\n Hay algunas celdas visitadas que no tienen bien asignado el segmentId.\n");
			//fflush(0);
        }
    }
}

void traducePartialSegmentationOpt(Modelo& m, binding* bd, vector<int>& traductionTable)
{
    // Recorrer cada vertice de la maya con un iterador y clasificar los vertices.
	for(int VertIdx = 0; VertIdx < bd->pointData.size(); VertIdx++ )
	{
        // Assegurar que los indices guardados en los vertices de la maya estan bien.
        assert(VertIdx >= 0 && VertIdx < m.vn());
        PointData& pd = bd->pointData[VertIdx];

        if(pd.segmentId>=0)
        {
			int nodeId = pd.segmentId % CROP_NODE_ID;
            pd.ownerLabel = traductionTable[nodeId];
        }
    }
}

void propagateHierarchicalSkinningOpt(Modelo& model, binding* bd, DefGraph& graph, 
									  DefGroup& group, int& times, 
									  vector<float>& weightsT1, 
									  vector<float>& weightsT2, 
									  vector<unsigned int>& indicesToCompute,
									  int surfaceIdx)
{
	times++;
	// The process goes down from every root joint and triggers a family fight for the influence.  
	//clock_t begin, end;

	if (group.relatedGroups.size() == 0) return;

	// 1. Domain initzialization for this process step
	//if(VERBOSE)printf("1. Domain initzialization: ");fflush(0);

	initDomainOpt(model, bd, group.nodeId, surfaceIdx);

	// 2. Establish every joint influence region
	//if(VERBOSE)printf("2. Volume segmentation");
	map<int, int> traductionTable;
	vector<int> segmentationIds;

	for(unsigned int id = 0; id < graph.deformers.size(); id++)
		traductionTable[graph.deformers[id]->nodeId] = graph.deformers[id]->boneId;

	// Adding the father to the fight
	//createTraductionTable(&group, traductionTable, group.nodeId, true);
	//segmentationIds.push_back(group.nodeId);

	// Adding the childs to the fight
	for(unsigned int i = 0; i< group.relatedGroups.size(); i++)
	{
		// Preparamos la traduccion para el grid.
		createTraductionTable(group.relatedGroups[i], traductionTable, group.relatedGroups[i]->nodeId);
        segmentationIds.push_back(group.relatedGroups[i]->nodeId);
	}

	// 2.b. Add also the father for fighting (TODEBUG IN TEST-TIME)
	createTraductionTable(&group, traductionTable, group.nodeId, true);

	// 2.c. We translate all the cells according to its node influencer.
	//if(VERBOSE)printf("2.c Translation\n");fflush(0);
	traducePartialSegmentation(model, bd, traductionTable, surfaceIdx);
	traductionTable.clear();

	for(unsigned int i = 0; i< segmentationIds.size(); i++)
	{
		//SmoothFromSegment(model, bd, graph.defGroupsRef[segmentationIds[i]], segmentationIds[i]);
		SmoothFromSegmentOpt(model, bd, graph.defGroupsRef[segmentationIds[i]], 
							 segmentationIds[i], weightsT1, weightsT2, indicesToCompute, surfaceIdx);
	}

	// 5. Normalizaci�n de pesos basados en el dominio
	// Se basa en los vectores auxInfluences.
	//if(VERBOSE)printf("5. Weights Normalization By Domain\n");fflush(0);
    normalizeWeightsByDomain(bd, surfaceIdx);

	for(unsigned int i = 0; i< group.relatedGroups.size(); i++)
	{
		//propagateHierarchicalSkinning(model,  bd, graph, *group.relatedGroups[i]);
		propagateHierarchicalSkinningOpt(model,  bd, graph, *group.relatedGroups[i], 
										 times, weightsT1, weightsT2, indicesToCompute, surfaceIdx);
	}
}

void clearOlderComputations(Modelo& m, int surfIdx)
{
	// Clear all older computed influences
	int totalValues = m.bind->surfaces[surfIdx].nodes.size();
	int blocks = 8;
	int valuesPerBlock = (int)ceil((float)totalValues / (float)blocks);

	#pragma opm parallel for
	for (int blockIdx = 0; blockIdx < blocks; blockIdx++)
	{
		int iniBlock = blockIdx*valuesPerBlock;
		int finBlock = (blockIdx+1)*valuesPerBlock;

		for (int ptIdx = iniBlock; ptIdx < finBlock; ptIdx++)
		{
			if (ptIdx >= m.bind->surfaces[surfIdx].nodes.size()) continue;

			PointData& pd = m.bind->pointData[m.bind->surfaces[surfIdx].nodes[ptIdx]->id];

			pd.influences.clear();
			pd.auxInfluences.clear();

			for (int i = 0; i < pd.secondInfluences.size(); i++)
				pd.secondInfluences[i].clear();

			pd.secondInfluences.clear();

			pd.domain = 0;
			pd.domainId = -1;
			pd.ownerLabel = -1;
			pd.ownerWeight = 0;
			pd.tempOwnerWeight = 0;

			pd.itPass = 0;
			pd.validated = false;
			pd.assigned = false;
		}

	}
	

	//TODEBUG:
	// Other actions for cleaning and setting up the data.
}

void initDomain(Modelo& m, binding* bd, int domainId_init)
{
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
                // In this case there is NO influence over this cell from domainId node
				// and it is not battled by the childs
                pd.domain = 0;
                pd.domainId = domainId_init;
            }
        }

        pd.itPass = 0;
		pd.auxInfluences.clear();
    }
}

void initDomainOpt(Modelo& m, binding* bd, int domainId_init, int surfaceIdx)
{
    // Assegurar que los indices guardados en los vertices de la maya estan bien.
	// Clear all older computed influences
	int totalValues = bd->surfaces[surfaceIdx].nodes.size();
	int blocks = 8;
	int valuesPerBlock = (int)ceil((float)totalValues / (float)blocks);

	#pragma opm parallel for
	for (int blockIdx = 0; blockIdx < blocks; blockIdx++)
	{
		int iniBlock = blockIdx*valuesPerBlock;
		int finBlock = (blockIdx + 1)*valuesPerBlock;

		for (int VertIdx = iniBlock; VertIdx < finBlock; VertIdx++)
		{
			if (VertIdx >= bd->surfaces[surfaceIdx].nodes.size()) continue;

			PointData& pd = bd->pointData[bd->surfaces[surfaceIdx].nodes[VertIdx]->id];

			// These are the temporary data structures for this iteration.
			pd.auxInfluences.clear();
			pd.ownerLabel = -1;

			// We set the domain parameters for this iteration
			if (domainId_init < 0)
			{
				// This is the base case, there is no domain restriction.
				pd.domain = 1;
				pd.domainId = domainId_init;
			}
			else
			{
				int idx = findWeight(pd.influences, domainId_init);
				if (idx >= 0)
				{
					// In this case there is influence over this cell from domainId node.
					pd.domain = pd.influences[idx].weightValue;
					pd.domainId = domainId_init;
				}
				else
				{
					// In this case there is NO influence over this cell from domainId node
					// and it is not battled by the childs
					pd.domain = 0;
					pd.domainId = domainId_init;
				}
			}
		}
	}
}

bool ComputeEmbeddingWithBD(Modelo& model, bool withPatches)
{
	binding* bds = model.bind;

	bds->BihDistances.resize(bds->surfaces.size());
	bds->A.resize(bds->surfaces.size());

	for(int surf = 0; surf < bds->surfaces.size(); surf++)
	{
		// Computation indices
		vector<int> indices;
		indices.resize(bds->surfaces[surf].nodes.size());
		for(int idx = 0; idx < bds->surfaces[surf].nodes.size(); idx++)
		{
			//indices.push_back(idx);
			//indices.push_back(bds[bind]->pointData[idx].node->id);
			indices[idx] = bds->surfaces[surf].nodes[idx]->pieceId;
		}

		printf("\n\nCalculo de A para embeding: [%d puntos]\n", bds->surfaces[surf].nodes.size());
		fflush(0);

		//TODEBUG: BihDistances with matrix approach... uggh!
		//computeBDBinding( model, bds, indices, bds->A[surf], withPatches);
		
		printf("\n\nComputado\n", bds->surfaces[surf].nodes.size());
		// Old computations
		
		printf("antes de nada\n");
		symMatrixLight dists;
		bindingBD( model, bds, &(bds->surfaces[surf]), indices, dists, withPatches);
		double maxValue = 0;
		for(int i = 0; i < dists.size; i++)
		{
			for(int j = i; j< dists.size; j++)
			{
				if(dists.get(i,j) != dists.get(i,j))
					int pararseme = 0;

				maxValue = max(maxValue, dists.get(i,j));
			}
		}

		printf("MaxValue:%f\n", maxValue);
		double scaleFactor = pow(10,6)/ maxValue;
		printf("ScaleFactor:%f\n", scaleFactor);

		bds->A[surf].resize(dists.size,dists.size);
		for(int i = 0; i < dists.size; i++)
		{
			for(int j = i; j< dists.size; j++)
			{
				float tempValue = dists.get(i,j)*scaleFactor;
				bds->A[surf](i,j) = tempValue;
				bds->A[surf](j,i) = tempValue;
			}
		}

	}

	printf("Fin computacion\n");

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
	if(bindSize != m.bind->surfaces.size())
		return false;

	//m.bind->BihDistances.resize(m.bind->surfaces.size());
	m.bind->A.resize(m.bind->surfaces.size());
	
	for(int bind = 0; bind < m.bind->surfaces.size(); bind++)
	{
		int pointsSize = 0;
		finbin.read( (char*) &pointsSize,  sizeof(int) );

		// Debe corresponder el numero de puntos
		if(pointsSize != m.bind->surfaces[bind].nodes.size())
			return false;

		//m.bind->BihDistances[bind].resize(pointsSize);
		m.bind->A[bind].resize(pointsSize,pointsSize);

		for(int row = 0; row < pointsSize; row++)
		{
			for(int col = row; col < pointsSize; col++)
			{
				float value = 0;
				finbin.read( (char*) &value,  sizeof(float) );

				//m.bind->BihDistances[bind].set(row,col,value);
				m.bind->A[bind](row,col) = value;
				m.bind->A[bind](col,row) = value;
			}
		}
	}
	
	finbin.close();

    printf("Cargado---\n");
	return true;
}

void propagateHierarchicalSkinningOpt(Modelo& model, binding* bd, DefGraph& graph, int surfIdx)
{
	clock_t ini00_ini = clock();
	clearOlderComputations(model, surfIdx);
	clock_t ini00_fin = clock();

	// 1. Establecemos el dominio
	//if(VERBOSE) printf("1. Domain initzialization\n");fflush(0);

	clock_t ini01_ini = clock();
	initDomainOpt(model, model.bind, FIRST_ITERATION, surfIdx);
	clock_t ini01_fin = clock();

	// 2. Segmentamos para todos los esqueletos en la escena
	//if(VERBOSE)printf("2. Volume segmentation\n");fflush(0);

	clock_t ini02_ini = clock();
	int idsSize = graph.deformers.size();

	vector<int> segmentationIds;
	map<int, int> traductionTable;
	
	for(unsigned int id = 0; id < idsSize; id++)
	{
		int tempId = graph.deformers[id]->nodeId;//%CROP_NODE_ID;
		traductionTable[tempId] = graph.deformers[id]->boneId;
	}

	for(unsigned int i = 0; i< graph.roots.size(); i++)
	{
		// Preparamos la traduccion para el grid.
		createTraductionTable(graph.roots[i], traductionTable, graph.roots[i]->nodeId);
        segmentationIds.push_back(graph.roots[i]->nodeId);
	}

	traducePartialSegmentation(model, bd, traductionTable, surfIdx);
    traductionTable.clear();
	clock_t ini02_fin = clock();

	// 3. Smooth entre hijos cortando seg�n el dominio
	//if(VERBOSE){printf("3. Segments smoothing\n");fflush(0);}

	clock_t ini03_ini = clock();
	vector<float> weightsT1;
	vector<float> weightsT2;
	vector<unsigned int> indicesToCompute;

	int numOfPoints = bd->surfaces[surfIdx].nodes.size();

	// Inicializaci�n de datos.
	weightsT1.resize(numOfPoints,0);
	weightsT2.resize(numOfPoints,0);
	indicesToCompute.resize(numOfPoints, -1);

	clock_t ini03_fin = clock();

	clock_t ini033_ini = clock();
	for(unsigned int i = 0; i< segmentationIds.size(); i++)
	{
		//printf("Segmentation %d of %d\n", i, segmentationIds.size());
		SmoothFromSegmentOpt(model, bd, graph.defGroupsRef[segmentationIds[i]], 
							segmentationIds[i], weightsT1, weightsT2, indicesToCompute, surfIdx);
	}
	clock_t ini033_fin = clock();

	clock_t ini04_ini = clock();
    normalizeWeightsByDomain(bd, surfIdx);
	clock_t ini04_fin = clock();

	clock_t ini044_ini = clock();
	cleanZeroInfluences(bd);
	clock_t ini044_fin = clock();

	int times = 1;

	clock_t ini05_ini = clock();
	for(unsigned int i = 0; i< graph.roots.size(); i++)
	{
        propagateHierarchicalSkinningOpt( model,  bd, graph, 
										  *graph.roots[i], times, 
										  weightsT1, weightsT2, 
										  indicesToCompute, surfIdx);
	}
	clock_t ini05_fin = clock();

	clock_t ini06_ini = clock();
	weightsT1.clear();
	weightsT2.clear();
	indicesToCompute.clear();
	clock_t ini06_fin = clock();

	/*
	printf("\n\n\nA. Tiempos de propagate hierarchicaly\n");
	printf("B. Clean old data %f\n", ((double)-(ini00_ini - ini00_fin)) / CLOCKS_PER_SEC * 1000);
	printf("C. initDomain %f\n", ((double)-(ini01_ini - ini01_fin)) / CLOCKS_PER_SEC * 1000);
	printf("D. Traduce vectors %f\n", ((double)-(ini02_ini - ini02_fin)) / CLOCKS_PER_SEC * 1000);
	printf("E. resize vectors %f\n", ((double)-(ini03_ini - ini03_fin)) / CLOCKS_PER_SEC * 1000);
	printf("E. Smooth from segment %f\n", ((double)-(ini033_ini - ini033_fin)) / CLOCKS_PER_SEC * 1000);
	printf("F. normalize by domain %f\n", ((double)-(ini04_ini - ini04_fin)) / CLOCKS_PER_SEC * 1000);
	printf("F. clean zero influences %f\n", ((double)-(ini044_ini - ini044_fin)) / CLOCKS_PER_SEC * 1000);
	printf("G. propagateHierarchicalSkinning %f\n", ((double)-(ini05_ini - ini05_fin)) / CLOCKS_PER_SEC * 1000);
	printf("H. Veces: %d, en media: %f del total de:%f\n\n\n", times, ((double)-(ini05_ini - ini05_fin)) / CLOCKS_PER_SEC * 1000 / times, ((double)-(ini05_ini - ini05_fin)) / CLOCKS_PER_SEC * 1000);
	printf("G. clear %f\n", ((double)-(ini06_ini - ini06_fin)) / CLOCKS_PER_SEC * 1000);
	*/

}

void propagateHierarchicalSkinning(Modelo& model, binding* bd, DefGraph& graph, DefGroup& dg)
{

}

void propagateHierarchicalSkinning(Modelo& model, binding* bd, DefGraph& graph)
{
	clearOlderComputations(model, 0);

	// 1. Establecemos el dominio
	if(VERBOSE) printf("1. Domain initzialization\n");fflush(0);
	initDomain(model, model.bind, FIRST_ITERATION);

	// 2. Segmentamos para todos los esqueletos en la escena
	if(VERBOSE)printf("2. Volume segmentation\n");fflush(0);

	map<int, int> traductionTable;
	vector<int> segmentationIds;

	for(unsigned int id = 0; id < graph.deformers.size(); id++)
		traductionTable[graph.deformers[id]->nodeId] = graph.deformers[id]->boneId;

	for(unsigned int i = 0; i< graph.roots.size(); i++)
	{
		// Preparamos la traduccion para el grid.
		createTraductionTable(graph.roots[i], traductionTable, graph.roots[i]->nodeId);
        segmentationIds.push_back(graph.roots[i]->nodeId);
	}

	traducePartialSegmentation(model, bd, traductionTable, 0);
    traductionTable.clear();

	// 3. Smooth entre hijos cortando seg�n el dominio
	if(VERBOSE){printf("3. Segments smoothing\n");fflush(0);}

	for(unsigned int i = 0; i< segmentationIds.size(); i++)
	{
		SmoothFromSegment(model, bd, graph.defGroupsRef[segmentationIds[i]], segmentationIds[i]);
	}

	// 5. Normalizaci�n de pesos basados en el dominio
	// Se basa en los vectores auxInfluences.
	if(VERBOSE)printf("5. Weights Normalization By Domain\n");fflush(0);
    normalizeWeightsByDomain(bd, 0);

	cleanZeroInfluences(bd);

	for(unsigned int i = 0; i< graph.roots.size(); i++)
	{
        propagateHierarchicalSkinning( model,  bd, graph, *graph.roots[i]);
	}
}

// TODEBUG: ensure the process of disconect deformers from the process.
void segmentModelFromDeformersOpt(  Modelo& model, 
									binding* bd, 
									DefGraph& graph, 
									MatrixXf& subDistances, 
									map<int, double>& precompDistancesMap,
									map<int, int>& matrixDefReference,
									map<int, bool>& defNodeDirtyBit,
									map<int, bool>& defNodeisInside,
									vector<int>& lastPostions,
									MatrixXf& computedDistances,
									int computationSize,
									int surfIdx)
{
    // m, bb->intPoints, bb->embeddedPoints
	vector< DefNode* >& deformers = graph.deformers;
	SurfaceGraph& surfGraph = bd->surfaces[surfIdx];
	
	map<int, bool> dirtyDeformers;

	// Ensure data consistency
	//assert(bd->pointData.size() == model.vn());

	int dirtyNodeCount = 0;
	int noFreeNodeCount = 0;
	int defNodeIsInsideCount = 0; 

	// Get wich deformers needs to be updated
	int dirtyCount = 0; 
	for(int deformerId = 0; deformerId < deformers.size(); deformerId++ )
	{
		int defIdx = deformers[deformerId]->nodeId;

		if (defNodeDirtyBit[defIdx]) dirtyNodeCount++;
		if (!deformers[deformerId]->freeNode) noFreeNodeCount++;
		if (defNodeisInside[defIdx]) defNodeIsInsideCount++;

		dirtyDeformers[defIdx] = defNodeDirtyBit[defIdx] && !deformers[deformerId]->freeNode && defNodeisInside[defIdx];
		
		if (dirtyDeformers[defIdx]) dirtyCount++;
	}
	
	//printf("Contadores: dirty:%d, noFree:%d, inside:%d , total:%d\n", dirtyNodeCount, noFreeNodeCount, defNodeIsInsideCount, deformers.size());

	VectorXf precomputedDistances(matrixDefReference.size());
	VectorXf defExpansion(matrixDefReference.size());

	int minCol = deformers.size();
	int maxCol = -1;
	for(int i = 0; i< deformers.size(); i++)
	{
		int nodeId = deformers[i]->nodeId;
		int matrixCol = matrixDefReference[nodeId];
		precomputedDistances[matrixCol] = precompDistancesMap[nodeId];
		defExpansion[matrixCol] = deformers[i]->expansion;

		if (!deformers[i]->freeNode && defNodeisInside[deformers[i]->nodeId])
		{
			minCol = min(minCol, matrixCol);
			maxCol = max(maxCol, matrixCol);
		}
	}

	//printf("Rango: de col %d a %d \n", minCol, maxCol);

	if (maxCol - minCol < 0)
	{
		// En este caso no tenemos que calcular nada
		return;
	}

	int dirtySizeComp = maxCol - minCol + 1;

	// Updates all the points that were linked to a dirty deformer
	// Marcamos los puntos que deben recalcularse
	int countPointsUpdate = 0;
	int countProblematic = 0;
	//#pragma omp parallel for
	for (int pointId = 0; pointId < surfGraph.nodes.size(); pointId++)
	{
		PointData& pd = bd->pointData[surfGraph.nodes[pointId]->id];
		if (pd.segmentId >= 0)
		{
			pd.assigned = true;
			if (dirtyDeformers[pd.segmentId] || graph.defNodesRef[pd.segmentId]->freeNode || !defNodeisInside[pd.segmentId])
			{
				pd.segmentDistance = 99999999;
				pd.segmentId = -1;
				pd.assigned = false;
				countPointsUpdate++;
			}
		}
		else
		{
			pd.assigned = false;
			countPointsUpdate++;
		}
	}

	//printf("Actualizamos %d puntos\n", countPointsUpdate);

	// Updates all the points comparing with all the dirty deformers
	// Evaluacion de columnas... hay calculos que no se han guardado y podrian valer
	int countDeformersComputation = 0; 
	//#pragma omp parallel for
	for(int deformerId = 0; deformerId < deformers.size(); deformerId++ )
	{
		DefNode* def = deformers[deformerId];
		if (dirtyDeformers[def->nodeId] && defNodeisInside[def->nodeId]/*!defNodeisInside[def->nodeId]*/)
		{
			float expansion = def->expansion;
			float precompDist = precompDistancesMap[def->nodeId];

			int realDefId = matrixDefReference[deformers[deformerId]->nodeId];

			VectorXf tempValues(surfGraph.nodes.size());
			tempValues.fill(precomputedDistances[realDefId]);

			computedDistances.col(realDefId) = ((subDistances.col(realDefId)*2) - tempValues);
			VectorXf newDistances = computedDistances.col(realDefId) / expansion;

			// este nodo tiene que pelear por todos los puntos.
			for(int pointId = 0; pointId < surfGraph.nodes.size(); pointId++ )
			{
				//#pragma omp critical
				{
					PointData& pd = bd->pointData[surfGraph.nodes[pointId]->id];
					float nd = newDistances[pointId];

					if (nd < 0)
					{
						//printf("\n\n\n\n\nUna distancia negativa... no es normal!\n\n\n\n\n");
						continue;
					}

					if (pd.segmentId < 0 || nd < pd.segmentDistance)
					{
						pd.segmentDistance = nd;
						pd.segmentId = def->nodeId;

						//if (pd.segmentId != def->nodeId)
						//{
							//pd.assigned = true;
						//}
					}
				}
			}

			countDeformersComputation++;
		}
	}

	//printf("dirtyCount: %d\n", dirtyCount);
	//printf("Rango de computo: %d deformers\n", countDeformersComputation);
	/*
	countPointsUpdate = 0;
	for (int pointId = 0; pointId < surfGraph.nodes.size(); pointId++)
	{
		PointData& pd = bd->pointData[surfGraph.nodes[pointId]->id];
		if (!pd.assigned)
			countPointsUpdate++;
	}
	*/

	//printf("Quedan por actualizar %d puntos\n", countPointsUpdate);

	//printf("Hemos evaluado %d columnas\n", countDeformersComputation);


	// Al ser un calculo progresivo, no es necesario este paso
	// solo se daria si se modifica la expansion, y en ese caso ira
	// por otra funcion... ahorramos mucho calculo.
	
	// Evaluate the rest of the points that are not assigned
	// Evaluacion por filas

	// TO_DEBUG: es posible que se pueda eliminar
	int countRestPoints = 0;
	int countSpecialCases = 0;
	int changesInlastPoint = 0;
	//#pragma omp parallel for
	for(int pointId = 0; pointId < surfGraph.nodes.size(); pointId++ )
    {
		PointData& pd = bd->pointData[surfGraph.nodes[pointId]->id];
		if (!pd.assigned || pd.segmentId < 0)
		{
			float distance = 99999999999;
			int newSegmentId = -1;

			VectorXf subDistRow = subDistances.row(pointId).segment(minCol, dirtySizeComp);
			subDistRow *= 2;
			computedDistances.row(pointId).segment(minCol, dirtySizeComp) = subDistRow - precomputedDistances.segment(minCol, dirtySizeComp);

			// TO_DEBUG
			VectorXf tempDistance(subDistRow.rows());
			for (int tempDistanceIdx = 0; tempDistanceIdx < tempDistance.rows(); tempDistanceIdx++)
			{
				tempDistance[tempDistanceIdx] = computedDistances(pointId, tempDistanceIdx) / defExpansion[tempDistanceIdx];
			}

			distance = tempDistance.minCoeff(&newSegmentId);

			if (distance < 0)
			{
				//printf("\n\n\n\n\nUna distancia negativa... no es normal!\n\n\n\n\n\n");
				continue;
			}

			if(newSegmentId >= 0)
			{
				pd.segmentId = lastPostions[newSegmentId];
				pd.segmentDistance = distance;
				pd.assigned = true;
			}
		}
	}

	/*
	countPointsUpdate = 0;
	for (int pointId = 0; pointId < surfGraph.nodes.size(); pointId++)
	{
		PointData& pd = bd->pointData[surfGraph.nodes[pointId]->id];
		if (!pd.assigned)
			countPointsUpdate++;
	}

	printf("FInalmente quedan por actualizar %d puntos!!!!\n", countPointsUpdate);
	*/
	
}


// TODEBUG: ensure the process of disconect deformers from the process.
void segmentModelFromDeformers(Modelo& model, binding* bd, DefGraph& graph)
{
	// REMOVE FUNCTION
	assert(false);

    // m, bb->intPoints, bb->embeddedPoints
	vector< DefNode* >& deformers = graph.deformers;
	map<int, bool> dirtyDeformers;

	// Ensure data consistency
	assert(bd->pointData.size() == model.vn());

	// Get wich deformers needs to be updated
	for(int deformerId = 0; deformerId < deformers.size(); deformerId++ )
	{
		dirtyDeformers[deformers[deformerId]->nodeId] = deformers[deformerId]->segmentationDirtyFlag;
	}

	//int time1  = 0; int time2 = 0; int times = 0;

	// Updates all the points that were linked to a dirty deformer
	for(int pointId = 0; pointId < bd->pointData.size(); pointId++ )
    {
		PointData& pd = bd->pointData[pointId];
		if(dirtyDeformers[pd.segmentId])
		{
			// El punto debe ser debatido entre todos los deformadores.
			float distance = 99999999999;
			int newSegmentId = -1;
			for(int deformerId = 0; deformerId < deformers.size(); deformerId++ )
			{
				DefNode* def = deformers[deformerId];
				
				float newDistance = -BiharmonicDistanceP2P_sorted(def->MVCWeights, 
														 def->weightsSort, 
														 pointId, bd, 
														 def->expansion, 
														 def->precomputedDistances, 
														 def->cuttingThreshold);
				/*
				float newDistance = -BiharmonicDistanceP2P_sorted_analisis(def->MVCWeights, 
														 def->weightsSort, 
														 pointId, bd, 
														 def->expansion, 
														 def->precomputedDistances,
														 time1, time2, times,
														 def->cuttingThreshold);
														 */
				if(newSegmentId < 0 || distance > newDistance)
				{
					distance = newDistance;
					newSegmentId = def->nodeId;
				}
			}

			if(newSegmentId > 0)
			{
				if(distance<0)
					int j = 0;
				pd.segmentId = newSegmentId;
				pd.segmentDistance = distance;
			}
		}
	}

	// Updates all the points comparing with all the dirty deformers
	for(int deformerId = 0; deformerId < deformers.size(); deformerId++ )
	{
		DefNode* def = deformers[deformerId];
		if(dirtyDeformers[deformers[deformerId]->nodeId])
		{
			// este nodo tiene que pelear por todos los puntos.
			for(int pointId = 0; pointId < bd->pointData.size(); pointId++ )
			{
				PointData& pd = bd->pointData[pointId];

				
				float newDistance = -BiharmonicDistanceP2P_sorted(def->MVCWeights, 
														 def->weightsSort, 
														 pointId, bd, 
														 def->expansion, 
														 def->precomputedDistances, 
														 def->cuttingThreshold);
				/*
				float newDistance = -BiharmonicDistanceP2P_sorted_analisis(def->MVCWeights, 
														 def->weightsSort, 
														 pointId, bd, 
														 def->expansion, 
														 def->precomputedDistances, 
														 time1, time2, times,
														 def->cuttingThreshold);
														 */
														 
				if(pd.segmentId < 0 || newDistance < pd.segmentDistance)
				{
					pd.segmentDistance = newDistance;
					pd.segmentId = def->nodeId;
				}
			}
		}

		def->segmentationDirtyFlag = false;
	}

	//printf("\nTime for matrix multiplication: per node:%f, global:%f \n ", ((double)time1)/times/CLOCKS_PER_SEC, ((double)time1)/CLOCKS_PER_SEC);

	//printf("Time for scalar multiplication: per node:%f, global:%f \n ", ((double)time2)/times/CLOCKS_PER_SEC, ((double)time2)/CLOCKS_PER_SEC);
}

int indexOfNode(int nodeId, vector<DefNode>& nodes)
{
	for(int i = 0; i< nodes.size(); i++)
	{
		if(nodes[i].nodeId == nodeId) return i;
	}

	return -1;
}

bool isInTheBrach( DefGroup* defGroup, int idx)
{
	if(indexOfNode(idx, defGroup->deformers) >= 0)
		return true;

	for(int i = 0; i< defGroup->relatedGroups.size(); i++)
	{
		if(isInTheBrach( defGroup->relatedGroups[i], idx))
			return true;
	}

	return false;
}

bool getDefomersfromBranch( DefGroup* defGroup, vector<DefNode*>& deformers)
{
	for(int i = 0; i< defGroup->deformers.size(); i++)
		deformers.push_back(&defGroup->deformers[i]);

	for(int i = 0; i< defGroup->relatedGroups.size(); i++)
		getDefomersfromBranch(defGroup->relatedGroups[i], deformers);

	return true;
}

// The comptation of a node, paralellized by OMP...
void computeNodesOptimized_justExpansion(DefGraph& graph,
										Modelo& model,
										MatrixXf& MatrixWeights,
										MatrixXf& distancesTemp,
										MatrixXf& computedDistances,
										map<int, double>& precompDistances,
										map<int, int>& defNodeRef,
										map<int, bool>& defNodeDirtyBit,
										map<int, bool>& defNodeIsInside,
										int surfaceIdx)
{
	//printf("  --  EXPANSION ONLY MODE  --\n");

	// Inicialization
	int defNodesSize = graph.deformers.size();

	int minCol = graph.deformers.size();
	int maxCol = -1;
	for (int i = 0; i< graph.deformers.size(); i++)
	{
		int nodeId = graph.deformers[i]->nodeId;
		int matrixCol = defNodeRef[nodeId];

		if (!graph.deformers[i]->freeNode && defNodeIsInside[nodeId])
		{
			minCol = min(minCol, matrixCol);
			maxCol = max(maxCol, matrixCol);
		}
	}

	if (maxCol - minCol < 0)
	{
		// En este caso no tenemos que calcular nada
		return;
	}

	int dirtySize = maxCol - minCol + 1;

	// Place all the dirty computations at the end
	vector<int> lastPostions(defNodeRef.size());
	VectorXf defExpansion(defNodeRef.size());

	map<int, int>::iterator it = defNodeRef.begin();
	while (it != defNodeRef.end())
	{
		int nodeId = it->second;
		if (nodeId > defNodeRef.size())
		{
			printf("hay algun problema de indices\n");
			assert(false);
		}

		lastPostions[nodeId] = it->first;
		defExpansion[nodeId] = graph.defNodesRef[it->first]->expansion;

		it++;
	}

	map<int, bool> dirtyDeformers;

	int computationSize;

	for (int i = 0; i < lastPostions.size(); i++)
	{
		if (defNodeDirtyBit[graph.defNodesRef[lastPostions[i]]->nodeId])
		{
			if (!graph.defNodesRef[lastPostions[i]]->freeNode && defNodeIsInside[graph.defNodesRef[lastPostions[i]]->nodeId])
				dirtyDeformers[graph.defNodesRef[lastPostions[i]]->nodeId] = true;
		}

		if (!graph.defNodesRef[lastPostions[i]]->freeNode && defNodeIsInside[graph.defNodesRef[lastPostions[i]]->nodeId])
			computationSize++;
	}

	vector< DefNode* >& deformers = graph.deformers;
	MatrixXf& A = model.bind->A[surfaceIdx];
	SurfaceGraph& surfGraph = model.bind->surfaces[surfaceIdx];

	// Updates all the points that were linked to a dirty deformer
	// Marcamos los puntos que deben recalcularse
	//#pragma omp parallel for

	// Clean values
	for (int pointId = 0; pointId < surfGraph.nodes.size(); pointId++)
	{
		PointData& pd = model.bind->pointData[surfGraph.nodes[pointId]->id];
		pd.assigned = false;
		if (pd.segmentId >= 0)
		{
			if (dirtyDeformers[pd.segmentId] || graph.defNodesRef[pd.segmentId]->freeNode)
			{
				pd.segmentDistance = 99999999999;
				pd.segmentId = -1;

				pd.assigned = true;
			}
		}
	}

	//Assign new points to the nodes
	int countDeformersComputation = 0;
	for (int deformerId = 0; deformerId < deformers.size(); deformerId++)
	{
		DefNode* def = deformers[deformerId];
		int nodeId = def->nodeId;
		if (dirtyDeformers[nodeId])
		{
			float expansion = def->expansion;
			int realDefId = defNodeRef[nodeId];

			VectorXf newDistances = computedDistances.col(realDefId) / expansion;

			// este nodo tiene que pelear por todos los puntos.
			for (int pointId = 0; pointId < surfGraph.nodes.size(); pointId++)
			{
				PointData& pd = model.bind->pointData[surfGraph.nodes[pointId]->id];
				float nd = newDistances[pointId];

				if (nd <= pd.segmentDistance || pd.segmentId < 0)
				{
					pd.segmentDistance = nd;
					pd.segmentId = nodeId;
				}
			}

			countDeformersComputation++;
		}
	}

	// Evaluate the rest of the points that are not assigned

	// Evaluacion por filas
	int count = 0; 
	for(int pointId = 0; pointId < surfGraph.nodes.size(); pointId++)
	{
		PointData& pd = model.bind->pointData[surfGraph.nodes[pointId]->id];
		if (pd.assigned || pd.segmentId < 0)
		{
			// El punto debe ser debatido entre todos los deformadores.
			float distance = 99999999999;
			int newSegmentId = -1;

			VectorXf tempDistance = computedDistances.row(pointId).segment(minCol, dirtySize);
			for (int tempDistanceIdx = 0; tempDistanceIdx < tempDistance.size(); tempDistanceIdx++)
			{
				tempDistance[tempDistanceIdx] /= defExpansion[tempDistanceIdx];
			}
				
			distance = tempDistance.minCoeff(&newSegmentId);

			if (newSegmentId >= 0)
			{
				pd.segmentId = lastPostions[newSegmentId];
				pd.segmentDistance = distance;
			}
		}
	}
}


void orderDataForComputation(
							DefGraph& graph, 
							vector<int>& lastPostions, 
							map<int, bool>& defNodeDirtyBit, 
							MatrixXf& MatrixWeights, 
							MatrixXf& distancesTemp,
							MatrixXf& computedDistances,
							map<int, double>& precompDistances,
							map<int, int>& defNodeRef,
							map<int, bool>& defNodeIsInside,
							int& blockSize, int& iniBlock)
{


	// Swap the elements, no matter the sort, just dirty in one hand and computed in the other
	// 1. We allocate all the dirty anf free nodes in the right hand, and computed ones in the left
	int idx1 = 0;
	int idx2 = lastPostions.size() - 1;
	int swapCount1 = 0;
	int swapCount2 = 0;
	while (idx2 > idx1)
	{
		// We move the indexes
		while (idx1 < lastPostions.size() && !defNodeDirtyBit[graph.defNodesRef[lastPostions[idx1]]->nodeId]) idx1++;
		while (idx2 >= 0 && defNodeDirtyBit[graph.defNodesRef[lastPostions[idx2]]->nodeId]) idx2--;

		if (idx1 == lastPostions.size() || idx2< 0 || idx2 < idx1) break;

		// SWAP
		int defNodeId001 = graph.defNodesRef[lastPostions[idx1]]->nodeId;
		int defNodeId002 = graph.defNodesRef[lastPostions[idx2]]->nodeId;

		// Matrix of weights
		MatrixWeights.col(idx1).swap(MatrixWeights.col(idx2));

		// Matrix of subdistances
		distancesTemp.col(idx1).swap(distancesTemp.col(idx2));

		// Matrix of distances
		computedDistances.col(idx1).swap(computedDistances.col(idx2));

		// References
		defNodeRef[defNodeId002] = idx1;
		defNodeRef[defNodeId001] = idx2;

		// Indexes
		int temp = lastPostions[idx1];
		lastPostions[idx1] = lastPostions[idx2];
		lastPostions[idx2] = temp;

		swapCount1++;
	}

	// 2. Just working over the right hand group, We allocate all the dirty nodes in the left hand 
	// and free ones in the right. With this we only compute the dirty nodes and do all the 
	// computations over this submatrix
	idx1 = -1;
	for (int i = 0; i < lastPostions.size(); i++)
	{
		if (graph.defNodesRef[lastPostions[i]]->freeNode || !defNodeIsInside[graph.defNodesRef[lastPostions[i]]->nodeId])
		{
			idx1 = i;
			break;
		}
	}
	idx2 = lastPostions.size() - 1;

	if (idx1 < 0) idx1 = 0;

	while (idx2 > idx1)
	{
		// We move the indexes
		while (idx1 < lastPostions.size() && !(graph.defNodesRef[lastPostions[idx1]]->freeNode || !defNodeIsInside[graph.defNodesRef[lastPostions[idx1]]->nodeId])) idx1++;
		while (idx2 >= 0 && (graph.defNodesRef[lastPostions[idx2]]->freeNode || !defNodeIsInside[graph.defNodesRef[lastPostions[idx2]]->nodeId])) idx2--;

		if (idx1 == lastPostions.size() || idx2< 0 || idx2 < idx1) break;

		// SWAP
		int defNodeId001 = graph.defNodesRef[lastPostions[idx1]]->nodeId;
		int defNodeId002 = graph.defNodesRef[lastPostions[idx2]]->nodeId;

		// Matrix of weights
		MatrixWeights.col(idx1).swap(MatrixWeights.col(idx2));

		// Matrix of subdistances
		distancesTemp.col(idx1).swap(distancesTemp.col(idx2));

		// Matrix of distances
		computedDistances.col(idx1).swap(computedDistances.col(idx2));

		// References
		defNodeRef[defNodeId002] = idx1;
		defNodeRef[defNodeId001] = idx2;

		// Indexes
		int temp = lastPostions[idx1];
		lastPostions[idx1] = lastPostions[idx2];
		lastPostions[idx2] = temp;

		swapCount2++;
	}

	idx1 = -1; // first index: change from computed to not computed
	idx2 = -1; // second index: change from not free to free ones.
	for (int i = 0; i < lastPostions.size(); i++)
	{
		if (idx2 < 0 && (graph.defNodesRef[lastPostions[i]]->freeNode || !defNodeIsInside[graph.defNodesRef[lastPostions[i]]->nodeId]))
		{
			idx2 = i;
		}

		if (idx1 < 0 && defNodeDirtyBit[graph.defNodesRef[lastPostions[i]]->nodeId])
		{
			idx1 = i;
		}
	}
	if (idx2 < 0) idx2 = lastPostions.size();
	if (idx1 < 0) idx1 = lastPostions.size();

	blockSize = idx2 - idx1;
	iniBlock = idx1;
}


// The comptation of a node, paralellized by OMP...
void computeNodesOptimized( DefGraph& graph, 
							Modelo& model, 
							MatrixXf& MatrixWeights, 
							MatrixXf& distancesTemp,
							MatrixXf& computedDistances,
							map<int, double>& precompDistances, 
							map<int, int>& defNodeRef,
							map<int, bool>& defNodeDirtyBit,
							map<int, bool>& defNodeIsInside,
							int surfaceIdx)
{
	
	//printf("  --  ALL COMPUTATION MODE  --\n");

	auto ini = high_resolution_clock::now();

	// Inicialization
	int defNodesSize = graph.deformers.size();	

	// Place all the dirty computations at the end
	vector<int> lastPostions(defNodeRef.size());

	map<int, int>::iterator it = defNodeRef.begin();
	while(it != defNodeRef.end())
	{
		if(it->second > defNodeRef.size())
		{ 
			printf("hay algun problema de indices\n");
			assert(false);
		}

		lastPostions[it->second] = it->first;
		it++;
	}

	int blockSize = 0;
	int iniBlock = 0;
	orderDataForComputation(graph, lastPostions, defNodeDirtyBit,
							MatrixWeights, distancesTemp, computedDistances,
							precompDistances, defNodeRef, defNodeIsInside, blockSize, iniBlock);

	vector<int> defNodesToUpdate;

	for(int i = 0 ; i < lastPostions.size(); i++ )
	{
		if (defNodeDirtyBit[graph.defNodesRef[lastPostions[i]]->nodeId])
		{
			if (!graph.defNodesRef[lastPostions[i]]->freeNode && defNodeIsInside[graph.defNodesRef[lastPostions[i]]->nodeId])
				defNodesToUpdate.push_back(i);
		}

		// Limpiar calculos antiguos
		MatrixWeights.col(i).fill(0);

	}

	//printf("DefNodes to update: %d \n", defNodesToUpdate.size());

	int modelPoints = model.bind->surfaces[surfaceIdx].nodes.size();
	vector<PointStr> vertexes(modelPoints);

	for (int i = 0; i < vertexes.size(); i++)
	{
		vertexes[i].x = model.bind->surfaces[surfaceIdx].nodes[i]->position.x();
		vertexes[i].y = model.bind->surfaces[surfaceIdx].nodes[i]->position.y();
		vertexes[i].z = model.bind->surfaces[surfaceIdx].nodes[i]->position.z();
	}

	// MVC - MULTI-THREAD
	#pragma omp parallel for
	for(int defId = 0; defId < defNodesToUpdate.size(); defId++ )
	{
		DefNode* df = graph.defNodesRef[lastPostions[defNodesToUpdate[defId]]];
		
		// Optimized version for CPU computation
		mvcOpt_ext(df->pos, MatrixWeights, defNodesToUpdate[defId], model, surfaceIdx, vertexes);
		//mvcOpt(df->pos, MatrixWeights, defNodesToUpdate[defId], model, surfaceIdx);
	}	

	// TO_DEBUG
	// Evaluar si es posible hacer un calculo mas optimizado, hecho a mano,
	// Teniendo en cuenta que es la A simetrica y que es column mayor. Y que muchas veces sera
	// la parte final lo unico que queremos multiplicar.

	MatrixXf& A = model.bind->A[surfaceIdx];
	SurfaceGraph& surfGraf = model.bind->surfaces[surfaceIdx];

	if(defNodesToUpdate.size() > 0 )
	{
		distancesTemp.block(0, iniBlock, surfGraf.nodes.size(), blockSize) =
			A*MatrixWeights.block(0, iniBlock, surfGraf.nodes.size(), blockSize);

		/*
		int width = MatrixWeights.cols();
		int heigh = MatrixWeights.rows();
		#pragma omp parallel for
		for (int C_Col_Idx = 0; C_Col_Idx < width; C_Col_Idx++)
		{
			for (int C_Row_Idx = 0; C_Row_Idx < heigh; C_Row_Idx++)
			{
			
				distancesTemp(C_Row_Idx, C_Col_Idx) = A.col(C_Row_Idx).dot(MatrixWeights.col(C_Col_Idx));
			}
		}
		*/

	}

	for(int defId = 0; defId < defNodesToUpdate.size(); defId++ )
	{
		DefNode* df = graph.defNodesRef[lastPostions[defNodesToUpdate[defId]]];
		precompDistances[df->nodeId] = -1;
	}

	// TO_DEBUG -> podriamos optimizar tambien.
	#pragma omp parallel for
	for(int defId = 0; defId < defNodesToUpdate.size(); defId++ )
	{
		float value = distancesTemp.col(defNodesToUpdate[defId]).transpose()*MatrixWeights.col(defNodesToUpdate[defId]);
		DefNode* df = graph.defNodesRef[lastPostions[defNodesToUpdate[defId]]];
		precompDistances[df->nodeId] = value;
	}

	// This will be removed or solved... by now...rest in peace.
	int computationSize = blockSize;
	if(computationSize < 0) computationSize = defNodeRef.size();

	segmentModelFromDeformersOpt(model, model.bind, graph, distancesTemp, precompDistances, defNodeRef, defNodeDirtyBit,
								 defNodeIsInside, lastPostions, computedDistances, computationSize, surfaceIdx);

	if(VERBOSE_PROCESS)
	{
		auto total_fin = high_resolution_clock::now();

		// Calculo de tiempos-> milisegundos
		auto ticks01 = duration_cast<microseconds>(total_fin-ini) ;
		printf("\nTOTAL!!!: %fms\n\n", (double)ticks01.count() / 1000.0);

		/*
		printf("1. MVC: %fms -> media: %fms\n", 
									(double)ticks01.count()/1000.0, 
									(double)ticks01.count()/1000.0/defNodesToUpdate.size());
		printf("2. Matrix mult: %fms -> media: %fms\n", 
									(double)ticks02.count()/1000.0, 
									(double)ticks02.count()/1000.0/defNodesToUpdate.size());
		printf("3. Precomputed distances: %fms -> media: %fms\n", 
									(double)ticks03.count()/1000.0, 
									(double)ticks03.count()/1000.0/defNodesToUpdate.size());
		printf("4. Segmentation: %fms\n", (double)ticks04.count()/1000.0);
		*/

	}

}

void computeSecondaryWeightsOpt(Modelo& model, binding* bd, 
								DefGraph& graph, 
								MatrixXf& subdistances, 
								map<int, int>& idxOrder, 
								MatrixXf& computedDistances, 
								int surfIdx, 
								bool wideValueComputation)
{
    // No hay ningun binding
    if(!bd ) return;

	vector< DefNode >& points = bd->intPoints;

	#pragma omp parallel for
	for(int pt = 0; pt < bd->pointData.size(); pt++)
	{
		PointData& dp = bd->pointData[pt];
		dp.secondInfluences.resize(dp.influences.size());
		for(int infl = 0; infl< dp.influences.size(); infl++)
		{
			int idInfl = dp.influences[infl].label;
			DefGroup* group = graph.defGroupsRef[idInfl];
			dp.secondInfluences[infl].resize(group->relatedGroups.size());

			for(int childIdx = 0; childIdx < group->relatedGroups.size(); childIdx++)
			{
				// Vamos a hacer una proyecci�n tal cual
				Vector3d fin = group->relatedGroups[childIdx]->transformation->translation;
				Vector3d ini = group->transformation->translation;
				Vector3d dir = fin-ini;
				float norm  = dir.norm();
				Vector3d posDir = dp.node->position-ini;

				float projectionValue = ((dir/norm).dot(posDir))/norm ;

				if(projectionValue < 0 ) projectionValue = 0;
				else if (projectionValue > 1) projectionValue = 1;

				dp.secondInfluences[infl][childIdx].alongBone = projectionValue;

				/*
				float dist = 9999999;
				int nodeIdChildSegment = -1;
				for(int defNodeIdx = 0; defNodeIdx < group->deformers.size(); defNodeIdx++)
				{
					DefNode* def = &group->deformers[defNodeIdx];
					if(group->relatedGroups[childIdx]->nodeId != def->childBoneId)
						continue;

					float newDistance = -BiharmonicDistanceP2P_sorted(
										def->MVCWeights, 
										def->weightsSort, 
										pt, bd, 
										def->expansion, 
										def->precomputedDistances, 
										def->cuttingThreshold);

					if(nodeIdChildSegment < 0 || newDistance < dist)
					{
						nodeIdChildSegment = defNodeIdx;
						dist = newDistance;
					}
				}

				if(nodeIdChildSegment> 0)
				{
					DefNode* def = &group->deformers[nodeIdChildSegment];
					dp.secondInfluences[infl][childIdx].alongBone = def->ratio;
				}
				else
				{
					dp.secondInfluences[infl][childIdx].alongBone = 0.0;
				}
				*/
			}
		}
	}

	// Just the first part of the computation
	if(!wideValueComputation) return; 

	map<int, int> fromThisGroup;
	for(int defIdx = 0; defIdx < graph.deformers.size(); defIdx++)
		fromThisGroup[graph.deformers[defIdx]->nodeId] = -1;

	for(int defGroupIdx = 0; defGroupIdx < graph.defGroups.size(); defGroupIdx++)
	{
		DefGroup* defGroup = graph.defGroups[defGroupIdx];

		map<int, int> childsFromGroup;
		
		// Relacionamos idGrupo con el hijo 
		for(int defIdx = 0; defIdx < defGroup->relatedGroups.size(); defIdx++)
			childsFromGroup[defGroup->relatedGroups[defIdx]->nodeId] = defIdx;

		// Ponemos a cero el mapa
		for(int defIdx = 0; defIdx < graph.deformers.size(); defIdx++)
			fromThisGroup[graph.deformers[defIdx]->nodeId] = -1;

		// Asignamos los hijos
		for(int defIdx = 0; defIdx < defGroup->deformers.size(); defIdx++)
			fromThisGroup[defGroup->deformers[defIdx].nodeId] = childsFromGroup[defGroup->deformers[defIdx].childBoneId];

		// We get only this segment from the model and init values of segmentation
		vector<PointData*> pointsFromThisGroup(0);
		for(int ptIdx = 0; ptIdx< bd->pointData.size(); ptIdx++)
		{
			PointData &pd = bd->pointData[ptIdx];

			if(fromThisGroup[pd.segmentId] >= 0)
			{
				pd.auxInfluences.clear();
				pd.ownerLabel = fromThisGroup[pd.segmentId];
				pointsFromThisGroup.push_back(&pd);
			}
			else
			{
					pd.auxInfluences.clear();
					pd.ownerLabel = -1;
					//pointsFromThisGroup.push_back(&pd);
			}
		}

		// Smooth the values iterating and doing a normalization.
		for(int childIdx = 0; childIdx < defGroup->relatedGroups.size(); childIdx++)
		{
			SmoothFromSegment(model, bd, defGroup, childIdx);
		}
		
		
		for(int ptIdx = 0; ptIdx < pointsFromThisGroup.size(); ptIdx++)
		{
			PointData* pd = pointsFromThisGroup[ptIdx];

			float sum = 0;
			for(int i = 0; i< pd->auxInfluences.size(); i++)
				sum += pd->auxInfluences[i].weightValue;

			if(pd->auxInfluences.size() > 0 && sum!= 0)
			{
				for(int i = 0; i< pd->auxInfluences.size(); i++)
				{
					float value = pd->auxInfluences[i].weightValue/sum;
					int inflIdx = findWeight(pd->influences, defGroup->nodeId);
					if(inflIdx >= 0)
					{
						pd->secondInfluences[inflIdx][pd->auxInfluences[i].label].wideBone = value;
					}
					else
					{
						// La influencia deber�a estar asignada, 
						//ser�a un poco raro que no lo estuviera
						assert(false);
					}

				}
			}
			else if(sum == 0) 
			{
				// es un poco curioso que pase esto
				//assert(false);
			}
			else 
			{
				// En este caso no hacemos nada, en principio los pesos 
				// secundarios tendran valores por defecto, es decir 1.
			}
		}
	}
}

void computeSecondaryWeights(Modelo& model, binding* bd, DefGraph& graph)
{
    // No hay ningun binding
    if(!bd ) return;

	vector< DefNode >& points = bd->intPoints;
	//vector< vector<double> >& weights = bd->embeddedPoints;
	//vector< vector<int> >& sortedWeights = bd->weightsSort;
	//vector< vector<weight> >& weightsClean = bd->weightsFiltered;
	for(int pt = 0; pt < bd->pointData.size(); pt++)
	{
		PointData& dp = bd->pointData[pt];
		dp.secondInfluences.resize(dp.influences.size());
		for(int infl = 0; infl< dp.influences.size(); infl++)
		{
			int idInfl = dp.influences[infl].label;
			DefGroup* group = graph.defGroupsRef[idInfl];
			dp.secondInfluences[infl].resize(group->relatedGroups.size());

			for(int childIdx = 0; childIdx < group->relatedGroups.size(); childIdx++)
			{
				float dist = 9999999;
				int nodeIdChildSegment = -1;
				for(int defNodeIdx = 0; defNodeIdx < group->deformers.size(); defNodeIdx++)
				{
					DefNode* def = &group->deformers[defNodeIdx];
					if(group->relatedGroups[childIdx]->nodeId != def->childBoneId)
						continue;

					float newDistance = -BiharmonicDistanceP2P_sorted(
										def->MVCWeights, 
										def->weightsSort, 
										pt, bd, 
										def->expansion, 
										def->precomputedDistances, 
										def->cuttingThreshold);

					if(nodeIdChildSegment < 0 || newDistance < dist)
					{
						nodeIdChildSegment = defNodeIdx;
						dist = newDistance;
					}
				}

				if(nodeIdChildSegment> 0)
				{
					DefNode* def = &group->deformers[nodeIdChildSegment];
					dp.secondInfluences[infl][childIdx].alongBone = def->ratio;
				}
				else
				{
					dp.secondInfluences[infl][childIdx].alongBone = 0.0;
				}
			}
		}
	}

	/*
	if(false)
	{

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
			DefGroup* group = graph.defGroupsRef[idInfl];

			dp.secondInfluences[infl].resize(group->relatedGroups.size());
			vector<bool> assigned;
			assigned.resize(group->relatedGroups.size(), false);

			// Si tiene hijos o realmente encuentra el deformador.... hay que ver segmentId, que tal.
			int idxNodeAsignedAux = indexOfNode(dp.segmentId,group->deformers);

			if(idxNodeAsignedAux<0)
			{
				for(int childIdx = 0; childIdx < group->relatedGroups.size(); childIdx++)
				{
					if(isInTheBrach(group->relatedGroups[childIdx], dp.segmentId))
					{
						
						{
							// The point is asociated with some child, we can try to do a proyection over this bone
							// and get this value, the other points takes a 0.
							dp.secondInfluences[infl][childIdx].alongBone = 1.0;
							assigned[childIdx] = true;
						}

						break;
					}
				}
				continue;
			}
			
			// El Hijo que ya esta asignado
			DefNode* asignedNode = &group->deformers[idxNodeAsignedAux];
			for(int childIdx = 0; childIdx < group->relatedGroups.size(); childIdx++)
			{
				if(group->relatedGroups[childIdx]->nodeId == asignedNode->childBoneId)
				{
					dp.secondInfluences[infl][childIdx].alongBone = asignedNode->ratio;
					assigned[childIdx] = true;
				}
			}

			// Discretizamos el resto segun el hijo
			vector<DefNode*> defs;
			for(int idxNode = 0; idxNode < group->deformers.size(); idxNode++)
			{
				DefNode& node = group->deformers[idxNode];

				if(node.boneId != group->nodeId)
				{
					// En principio ninguno entrara por aqui
					continue;
				}

				if(node.childBoneId != asignedNode->childBoneId)
				{
					defs.push_back(&node);
				}
			}

			//Evaluaremos para cada hijo, cual es el mejor defNode y asigmanos su ratio
			if(group->relatedGroups.size()>1)
			{
				for(int childIdx = 0; childIdx < group->relatedGroups.size(); childIdx++)
				{
					if(assigned[childIdx])continue;

					float dist = 9999999;
					int nodeIdChildSegment = -1;
				
					for(int idxNode = 0; idxNode < defs.size(); idxNode++)
					{
						// Solo comparamos entre si los nodos que van al mismo hijo.
						if(defs[idxNode]->childBoneId != group->relatedGroups[childIdx]->nodeId )
							continue;

						DefNode* def = defs[idxNode];
						float newDistance = -BiharmonicDistanceP2P_sorted(def->MVCWeights, 
											def->weightsSort, 
											pt, bd, 
											def->expansion, 
											def->precomputedDistances, 
											def->cuttingThreshold);

						if(nodeIdChildSegment < 0)
						{
							nodeIdChildSegment = idxNode;
							dist = newDistance;
						}
						else if(newDistance < dist)
						{
							nodeIdChildSegment = idxNode;
							dist = newDistance;
						}
					}

					// Tenemos un elementos fuera, o no parece que no se acerca a nadie
					if(nodeIdChildSegment >= 0)
					{
						dp.secondInfluences[infl][childIdx].alongBone = defs[nodeIdChildSegment]->ratio;
					}
					else
					{
						printf("Tenemos un nodo que analizar\n"); fflush(0);
					}

				}
			}

			
		}
	}

	}
	*/

	map<int, int> fromThisGroup;
	for(int defIdx = 0; defIdx < graph.deformers.size(); defIdx++)
		fromThisGroup[graph.deformers[defIdx]->nodeId] = -1;

	for(int defGroupIdx = 0; defGroupIdx < graph.defGroups.size(); defGroupIdx++)
	{
		DefGroup* defGroup = graph.defGroups[defGroupIdx];

		map<int, int> childsFromGroup;
		
		// Relacionamos idGrupo con el hijo 
		for(int defIdx = 0; defIdx < defGroup->relatedGroups.size(); defIdx++)
			childsFromGroup[defGroup->relatedGroups[defIdx]->nodeId] = defIdx;

		// Ponemos a cero el mapa
		for(int defIdx = 0; defIdx < graph.deformers.size(); defIdx++)
			fromThisGroup[graph.deformers[defIdx]->nodeId] = -1;

		// Asignamos los hijos
		for(int defIdx = 0; defIdx < defGroup->deformers.size(); defIdx++)
			fromThisGroup[defGroup->deformers[defIdx].nodeId] = childsFromGroup[defGroup->deformers[defIdx].childBoneId];

		// We get only this segment from the model and init values of segmentation
		vector<PointData*> pointsFromThisGroup(0);
		for(int ptIdx = 0; ptIdx< bd->pointData.size(); ptIdx++)
		{
			PointData &pd = bd->pointData[ptIdx];

			if(fromThisGroup[pd.segmentId] >= 0)
			{
				pd.auxInfluences.clear();
				pd.ownerLabel = fromThisGroup[pd.segmentId];
				pointsFromThisGroup.push_back(&pd);
			}
			else
			{
					pd.auxInfluences.clear();
					pd.ownerLabel = -1;
					//pointsFromThisGroup.push_back(&pd);
			}
		}

		// Smooth the values iterating and doing a normalization.
		for(int childIdx = 0; childIdx < defGroup->relatedGroups.size(); childIdx++)
		{
			SmoothFromSegment(model, bd, defGroup, childIdx);
		}
		
		
		for(int ptIdx = 0; ptIdx < pointsFromThisGroup.size(); ptIdx++)
		{
			PointData* pd = pointsFromThisGroup[ptIdx];

			float sum = 0;
			for(int i = 0; i< pd->auxInfluences.size(); i++)
				sum += pd->auxInfluences[i].weightValue;

			if(pd->auxInfluences.size() > 0 && sum!= 0)
			{
				for(int i = 0; i< pd->auxInfluences.size(); i++)
				{
					float value = pd->auxInfluences[i].weightValue/sum;
					int inflIdx = findWeight(pd->influences, defGroup->nodeId);
					if(inflIdx >= 0)
					{
						pd->secondInfluences[inflIdx][pd->auxInfluences[i].label].wideBone = value;
					}
					else
					{
						// La influencia deber�a estar asignada, 
						//ser�a un poco raro que no lo estuviera
						assert(false);
					}

				}
			}
			else if(sum == 0) 
			{
				// es un poco curioso que pase esto
				//assert(false);
			}
			else 
			{
				// En este caso no hacemos nada, en principio los pesos 
				// secundarios tendran valores por defecto, es decir 1.
			}
		}
	}
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
		printf("Guardado embedding en fichero binario: %s\n", binFile);
	}

	if(ascii)
		fprintf(foutascii,"%d\n", model.bind->surfaces.size());
	else
	{
		int ss = model.bind->surfaces.size();
		foutbin.write((const char*) &ss, sizeof(int));
	}

	for(int surfaceIdx = 0 ; surfaceIdx < model.bind->surfaces.size(); surfaceIdx++)
	{
		// Number of nodes.
		if(ascii)
		{
			fprintf(foutascii,"%f\n", model.bind->surfaces[surfaceIdx].nodes.size());
		}
		else
		{
			int ss2 = model.bind->surfaces[surfaceIdx].nodes.size();
			foutbin.write((const char*) &ss2, sizeof(int));
		}

		// Matriz de distancias.
		int rowsNum = model.bind->A[surfaceIdx].rows();
		int colsNum = model.bind->A[surfaceIdx].cols();
		for(int row = 0; row < rowsNum; row++)
		{
			for(int col = row; col < colsNum; col++)
			{
				if(ascii)
					fprintf(foutascii,"%f\n", model.bind->A[surfaceIdx](row,col));
				else
				{
					float val = model.bind->A[surfaceIdx](row,col);
					foutbin.write((const char*) &val, sizeof(float));
				}
			}
		}
		/*for(int row = 0; row < model.bind->BihDistances[surfaceIdx].size; row++)
		{
			for(int col = row; col < model.bind->BihDistances[surfaceIdx].size; col++)
			{
				if(ascii)
					fprintf(foutascii,"%f\n", model.bind->BihDistances[surfaceIdx].get(row,col));
				else
				{
					double val = model.bind->BihDistances[surfaceIdx].get(row,col);
					foutbin.write((const char*) &val, sizeof(double));
				}
			}
		}*/
	}

	if(ascii)
		fclose(foutascii);
	else
		foutbin.close();
	
}