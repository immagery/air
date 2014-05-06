#include "AirSegmentation.h"

//#include <Computation\Segmentation.h>
#include <Computation\mvc_interiorDistances.h>
#include <Computation/BiharmonicDistances.h>

#include <DataStructures\InteriorDistancesData.h>

#include <DataStructures/Scene.h>

#include <cuda/cudaMgr.cuh>

#include <algorithm>

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
			graph.deformers[defId]->precomputedDistances = PrecomputeDistancesSingular_sorted(graph.deformers[defId]->MVCWeights, 
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
	propagateHierarchicalSkinningOpt(model, model.bind, graph);
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

void updateAirSkinningWithCuda(DefGraph& graph, Modelo& model)
{
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

void initSurfaceWeightsSmoothingOpt(Modelo& m, 
									binding* bd, 
									int nodeId, 
									vector<float>& weights,
									vector<float>& weightsAux,
									vector<unsigned int>& indicesToCompute,
									int& lastIndex)
{
	lastIndex = 0;
	int precindibleElems = 0;

	int length = bd->pointData.size();
	// recorrer todo el modelo e inicializar los datos para la transmision
	//int numThreads = omp_get_max_threads();
	//int range = (length/numThreads)+1;

	int idx = 0;
	//#pragma omp parallel shared(bd, weightsAux, weights, range) private(idx)
	{
		//#pragma omp for schedule(dynamic,1)
		for(idx = 0; idx < length; idx++ )
		{	
			PointData& pd = bd->pointData[idx];
			pd.node->visited = false;
			if(pd.ownerLabel != nodeId)
			{
				weightsAux[idx] = weights[idx] = 0.0;
			}
			else
			{
				weightsAux[idx] = weights[idx] = 1.0;
			}
		}
	}

	//#pragma omp parallel shared(bd, weightsAux, weights, range) private(idx)
	{
		// Seleccionamos para procesar solo lo necesario.
		//#pragma omp for schedule(dynamic,1) nowait
		for(idx = 0; idx < length; idx++ )
		{
			float value = weights[idx]; 
			PointData& pd = bd->pointData[idx];

			bool difValues = false;
			for(int n = 0; n< pd.node->connections.size() && !difValues; n++)
			{
				int neighId = pd.node->connections[n]->id;
				difValues |= (value != weights[neighId]);
			}

			if(difValues)
			{
				
				pd.node->visited = true;
				//#pragma omp critical
				{
				indicesToCompute[lastIndex] = idx;
				lastIndex++;
				}
			}
		}
	}

	//printf("[initSurfaceWeightsSmoothingOpt] \
			Hay %d elementos prescindibles de %d elemems, y podemos reorganizar este trozo \n", 
	//		precindibleElems, 
	//		lastIndex);
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
						  vector<float>& weightsT1, vector<float>& weightsT2, vector<unsigned int>& indicesToCompute)
{
	int lastIndex = 0;

    // Init model for weights smoothing
	//if(VERBOSE) printf("\n-- initCellWeightsSmoothing --\n");fflush(0);
	//clock_t ini00 = clock();
    initSurfaceWeightsSmoothingOpt(m, bd, frontId, weightsT1, weightsT2, indicesToCompute, lastIndex);
	//clock_t fin00 = clock();

    // Smoothing (sin corte jer�rquico)
	//if(VERBOSE) printf("-- Smoothing -- \n");fflush(0);

	// get the global or local smoothing value
	int smoothingPasses = group->smoothingPasses;
	float realSmooth = group->smoothPropagationRatio;

	//clock_t ini02 = clock();
    weightsSmoothing_opt(m, bd, realSmooth, 
						frontId, smoothingPasses, weightsT1, 
						weightsT2, indicesToCompute, lastIndex);
	//weightsNoSmooting(m, bd, front, frontId);

	//clock_t fin02 = clock();

	//printf("Valores de tiempo: preinit->%f  computo->%f\n", 
	//		((double)fin00-ini00)/CLOCKS_PER_SEC, 
	//		((double)fin02-ini02)/CLOCKS_PER_SEC); 
	//fflush(0);

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
						  int& lastIndex)

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

		int numElemems = (tempLastIndex/threads);
		#pragma omp for schedule(static, numElemems)
		for(int ptIdx = 0; ptIdx < tempLastIndex; ptIdx++)
        {
			int ownIdx = indicesToCompute[ptIdx];
			PointData& pd = bd->pointData[ownIdx];
			float value = 0;

			// Get the sum of values
			int cn = pd.node->connections.size();
			// Para cada elemento del frente realizamos la expansi�n.
			for(unsigned int neighboursIdx = 0; neighboursIdx < cn; neighboursIdx++)
			{
				int neighbourId = pd.node->connections[neighboursIdx]->id;
				value += (*ptWT1)[neighbourId];

				if(!pd.node->connections[neighboursIdx]->visited)
				{
					pd.node->connections[neighboursIdx]->visited = true;
					#pragma omp critical
					{
						indicesToCompute[lastIndex] = neighbourId;
						lastIndex++;
					}
				}
			}

			// Do the mean
			if(cn > 0) value /= cn;

			(*ptWT2)[ownIdx] = value;
        }

		// Change of array to compute weights
		vector<float>* tempVectPtr = ptWT1;
		ptWT1 = ptWT2;
		ptWT2 = tempVectPtr;
        iter++;
    }

	int numElemems2 = (bd->pointData.size()/threads);
	#pragma omp for schedule(static, numElemems2)
	for(int ptIdx = 0; ptIdx < bd->pointData.size(); ptIdx++)
    {
		PointData& pd = bd->pointData[ptIdx];

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

void traducePartialSegmentation(Modelo& m, binding* bd, map<int, int>& traductionTable)
{
    // Recorrer cada vertice de la maya con un iterador y clasificar los vertices.
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
            printf("Hay algunas celdas visitadas que no tienen bien asignado el segmentId.[ver traduceParcialSegmentation(...)]\n");
			fflush(0);
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

void propagateHierarchicalSkinningOpt(Modelo& model, binding* bd, DefGraph& graph, DefGroup& group, int& times, 
									  vector<float>& weightsT1, vector<float>& weightsT2, vector<unsigned int>& indicesToCompute)
{
	times++;
	// The process goes down from every root joint and triggers a family fight for the influence.  
	//clock_t begin, end;

	if (group.relatedGroups.size() == 0) return;

	// 1. Domain initzialization for this process step
	//if(VERBOSE)printf("1. Domain initzialization: ");fflush(0);

	initDomainOpt(model, bd, group.nodeId);

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
	traducePartialSegmentation(model, bd, traductionTable);
	traductionTable.clear();

	for(unsigned int i = 0; i< segmentationIds.size(); i++)
	{
		//SmoothFromSegment(model, bd, graph.defGroupsRef[segmentationIds[i]], segmentationIds[i]);
		SmoothFromSegmentOpt(model, bd, graph.defGroupsRef[segmentationIds[i]], segmentationIds[i], weightsT1, weightsT2, indicesToCompute);
	}

	// 5. Normalizaci�n de pesos basados en el dominio
	// Se basa en los vectores auxInfluences.
	//if(VERBOSE)printf("5. Weights Normalization By Domain\n");fflush(0);
    normalizeWeightsByDomain(bd);

	for(unsigned int i = 0; i< group.relatedGroups.size(); i++)
	{
		//propagateHierarchicalSkinning(model,  bd, graph, *group.relatedGroups[i]);
		propagateHierarchicalSkinningOpt(model,  bd, graph, *group.relatedGroups[i], times, weightsT1, weightsT2, indicesToCompute);
	}
}

void propagateHierarchicalSkinning(Modelo& model, binding* bd, DefGraph& graph, DefGroup& group)
{
	// The process goes down from every root joint and triggers a family fight for the influence.  
	clock_t begin, end;

	if (group.relatedGroups.size() == 0) return;

	// 1. Domain initzialization for this process step
	if(VERBOSE)printf("1. Domain initzialization: ");fflush(0);

	initDomain(model, bd, group.nodeId);

	// 2. Establish every joint influence region
	if(VERBOSE)printf("2. Volume segmentation");
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
	if(VERBOSE)printf("2.c Translation\n");fflush(0);
	traducePartialSegmentation(model, bd, traductionTable);
	traductionTable.clear();

	for(unsigned int i = 0; i< segmentationIds.size(); i++)
	{
		SmoothFromSegment(model, bd, graph.defGroupsRef[segmentationIds[i]], segmentationIds[i]);
	}

	// 5. Normalizaci�n de pesos basados en el dominio
	// Se basa en los vectores auxInfluences.
	if(VERBOSE)printf("5. Weights Normalization By Domain\n");fflush(0);
    normalizeWeightsByDomain(bd);

	for(unsigned int i = 0; i< group.relatedGroups.size(); i++)
	{
		propagateHierarchicalSkinning(model,  bd, graph, *group.relatedGroups[i]);
	}
}

void clearOlderComputations(Modelo& m)
{
	// Clear all older computed influences

	for(int ptIdx = 0; ptIdx< m.bind->pointData.size(); ptIdx++)
	{
		PointData& pd = m.bind->pointData[ptIdx];

		pd.influences.clear();
		pd.auxInfluences.clear();
		
		for(int i = 0; i< pd.secondInfluences.size(); i++)
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

void initDomainOpt(Modelo& m, binding* bd, int domainId_init)
{
    // Assegurar que los indices guardados en los vertices de la maya estan bien.
	int length = bd->pointData.size();
	//int numThreads = omp_get_max_threads();
	//int range = (length/numThreads)+1;

	int VertIdx = 0;
	//#pragma omp parallel shared(bd, domainId_init) private(VertIdx)
	{
		//#pragma omp for schedule(static,range) nowait
		for(VertIdx = 0; VertIdx < length; VertIdx++ )
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

			//pd.itPass = 0;
			//pd.auxInfluences.clear();
		}
	}
}

bool ComputeEmbeddingWithBD(Modelo& model, bool withPatches)
{
	binding* bds = model.bind;

	bds->BihDistances.resize(bds->surfaces.size());

	for(int surf = 0; surf < bds->surfaces.size(); surf++)
	{
		// Computation indices
		vector<int> indices;
		indices.resize(bds->surfaces[surf].nodes.size());
		for(int idx = 0; idx < bds->surfaces[surf].nodes.size(); idx++)
		{
			//indices.push_back(idx);
			//indices.push_back(bds[bind]->pointData[idx].node->id);
			indices[idx] = bds->surfaces[surf].nodes[idx]->id;
		}

		printf("\n\nCalculo de A para embeding: [%d puntos]\n", bds->surfaces[surf].nodes.size());
		fflush(0);

		//TODEBUG: BihDistances with matrix approach... uggh!
		bindingBD( model, bds, indices, bds->BihDistances[surf], withPatches);
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
	if(bindSize != m.bind->surfaces.size())
		return false;

	m.bind->BihDistances.resize(m.bind->surfaces.size());
	for(int bind = 0; bind < m.bind->surfaces.size(); bind++)
	{
		int pointsSize = 0;
		finbin.read( (char*) &pointsSize,  sizeof(int) );

		// Debe corresponder el numero de puntos
		if(pointsSize != m.bind->surfaces[bind].nodes.size())
			return false;

		m.bind->BihDistances[bind].resize(pointsSize);
		for(int row = 0; row < m.bind->BihDistances[bind].size; row++)
		{
			for(int col = row; col < m.bind->BihDistances[bind].size; col++)
			{
				double value = 0;
				finbin.read( (char*) &value,  sizeof(double) );
				m.bind->BihDistances[bind].set(row,col,value);
			}
		}
	}
	
	finbin.close();

    printf("Cargado---\n");
	return true;
}

void propagateHierarchicalSkinningOpt(Modelo& model, binding* bd, DefGraph& graph)
{
	//clock_t ini00 = clock();
	clearOlderComputations(model);

	// 1. Establecemos el dominio
	//if(VERBOSE) printf("1. Domain initzialization\n");fflush(0);

	initDomainOpt(model, model.bind, FIRST_ITERATION);

	// 2. Segmentamos para todos los esqueletos en la escena
	//if(VERBOSE)printf("2. Volume segmentation\n");fflush(0);

	int idsSize = graph.deformers.size();

	vector<int> segmentationIds;
	map<int, int> traductionTable;
	//vector<int> traductionTable(idsSize);
	//printf("Def Size:%d y %d nodos usados\n", graph.deformers.size(), scene::defNodeIds);
	
	for(unsigned int id = 0; id < idsSize; id++)
	{
		int tempId = graph.deformers[id]->nodeId;//%CROP_NODE_ID;
		traductionTable[tempId] = graph.deformers[id]->boneId;
	}

	//map<int, int> traductionTable;
	//for(unsigned int id = 0; id < graph.deformers.size(); id++)
	//	traductionTable[graph.deformers[id]->nodeId] = graph.deformers[id]->boneId;

	for(unsigned int i = 0; i< graph.roots.size(); i++)
	{
		// Preparamos la traduccion para el grid.
		createTraductionTable(graph.roots[i], traductionTable, graph.roots[i]->nodeId);
        segmentationIds.push_back(graph.roots[i]->nodeId);
	}
	//clock_t fin03 = clock();

	//clock_t ini04 = clock();
	traducePartialSegmentation(model, bd, traductionTable);
    traductionTable.clear();
	//clock_t fin04 = clock();

	// 3. Smooth entre hijos cortando seg�n el dominio
	//if(VERBOSE){printf("3. Segments smoothing\n");fflush(0);}

	vector<float> weightsT1;
	vector<float> weightsT2;
	vector<unsigned int> indicesToCompute;
	
	int numOfPoints = model.vn();

	// Inicializaci�n de datos.
	weightsT1.resize(numOfPoints,0);
	weightsT2.resize(numOfPoints,0);
	indicesToCompute.resize(numOfPoints, -1);

	//clock_t ini05 = clock();
	for(unsigned int i = 0; i< segmentationIds.size(); i++)
	{
		//printf("Segmentation %d of %d\n", i, segmentationIds.size());
		SmoothFromSegmentOpt(model, bd, graph.defGroupsRef[segmentationIds[i]], 
							segmentationIds[i], weightsT1, weightsT2, indicesToCompute);
	}
	//clock_t fin05 = clock();

	// 5. Normalizaci�n de pesos basados en el dominio
	// Se basa en los vectores auxInfluences.
	/*
	if(VERBOSE)
	{
		printf("5. Weights Normalization By Domain\n");
		fflush(0);
	}
	*/

	//clock_t ini06 = clock();
    normalizeWeightsByDomain(bd);
	//clock_t fin06 = clock();

	/*
	if(VERBOSE)
	{
		printf("6. Clean zero influences\n");
		fflush(0);
	}
	*/

	//clock_t ini07 = clock();
	cleanZeroInfluences(bd);
	//clock_t fin07 = clock();

	int times = 1;

	//clock_t ini08 = clock();
	for(unsigned int i = 0; i< graph.roots.size(); i++)
	{
        propagateHierarchicalSkinningOpt( model,  bd, graph, *graph.roots[i], times, weightsT1, weightsT2, indicesToCompute);
	}
	//clock_t fin08 = clock();

	weightsT1.clear();
	weightsT2.clear();
	indicesToCompute.clear();

	//printf("\n\n\nA. Tiempos de propagate hierarchicaly\n");
	//printf("B. Calculo de inicializacion %f\n", ((double)(fin03-ini00))/CLOCKS_PER_SEC);
	//printf("C. Traduce segmentation %f\n", ((double)(fin04-ini04))/CLOCKS_PER_SEC);
	//printf("D. Smooth from segment %f\n", ((double)(fin05-ini05))/CLOCKS_PER_SEC);
	//printf("E. normalize by domain %f\n", ((double)(fin06-ini06))/CLOCKS_PER_SEC);
	//printf("F. clean zero influences %f\n", ((double)(fin07-ini07))/CLOCKS_PER_SEC);
	//printf("G. propagateHierarchicalSkinning %f\n", ((double)(fin08-ini08))/CLOCKS_PER_SEC);
	//printf("H. Veces: %d, en media: %f\n\n\n", times, ((double)(fin08-ini08))/CLOCKS_PER_SEC/times);

}

void propagateHierarchicalSkinning(Modelo& model, binding* bd, DefGraph& graph)
{
	clearOlderComputations(model);

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

	traducePartialSegmentation(model, bd, traductionTable);
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
    normalizeWeightsByDomain(bd);

	cleanZeroInfluences(bd);

	for(unsigned int i = 0; i< graph.roots.size(); i++)
	{
        propagateHierarchicalSkinning( model,  bd, graph, *graph.roots[i]);
	}
}

// TODEBUG: ensure the process of disconect deformers from the process.
void segmentModelFromDeformersOpt(Modelo& model, binding* bd, DefGraph& graph, MatrixXf& subDistances)
{
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

	int time1  = 0; int time2 = 0; int times = 0;

	VectorXf precomputedDistances(deformers.size());
	VectorXf defExpansion(deformers.size());
	
	for(int i = 0; i< deformers.size(); i++)
	{
		precomputedDistances[i] = deformers[i]->precomputedDistances;
		defExpansion[i] = deformers[i]->expansion;
	}

	// Updates all the points that were linked to a dirty deformer
	// Evaluacion de filas
	for(int pointId = 0; pointId < bd->pointData.size(); pointId++ )
    {
		PointData& pd = bd->pointData[pointId];
		int deformerId = dirtyDeformers[pd.segmentId];
		if(dirtyDeformers[deformerId])
		{
			DefNode* def = deformers[deformerId];
			// El punto debe ser debatido entre todos los deformadores.
			float distance = 99999999999;
			int newSegmentId = -1;
			float preCompDistance = def->precomputedDistances;

			VectorXf tempDistance = ((subDistances.row(pd.segmentId)*-2)+precomputedDistances).cwiseQuotient(defExpansion);
			distance = tempDistance.minCoeff(&newSegmentId);

			if(newSegmentId > 0)
			{
				pd.segmentId = newSegmentId;
				pd.segmentDistance = distance;
			}
		}
	}

	
	// Updates all the points comparing with all the dirty deformers
	// Evaluacion de columnas... hay calculos que no se han guardado y podrian valer
	for(int deformerId = 0; deformerId < deformers.size(); deformerId++ )
	{
		DefNode* def = deformers[deformerId];
		if(dirtyDeformers[deformers[deformerId]->nodeId])
		{
			float expansion = def->expansion;
			float precompDist = def->precomputedDistances;

			VectorXf newDistances = ((subDistances.col(deformerId)*-2)+precomputedDistances)/expansion;

			// este nodo tiene que pelear por todos los puntos.
			for(int pointId = 0; pointId < bd->pointData.size(); pointId++ )
			{
				PointData& pd = bd->pointData[pointId];
				float nd = newDistances[pointId];								
														 
				if(pd.segmentId < 0 || nd < pd.segmentDistance)
				{
					pd.segmentDistance = nd;
					pd.segmentId = def->nodeId;
				}
			}
		}

		def->segmentationDirtyFlag = false;
	}
}


// TODEBUG: ensure the process of disconect deformers from the process.
void segmentModelFromDeformers(Modelo& model, binding* bd, DefGraph& graph)
{
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
						/*
						vector<DefNode*> deformersFromBranch;
						getDefomersfromBranch(group->relatedGroups[childIdx], deformersFromBranch);

						float distance = 999999;
						int assignedIdx = -1;
						for(int defbranchIdx = 0; defbranchIdx< deformersFromBranch.size(); defbranchIdx++)
						{
							DefNode* def = deformersFromBranch[defbranchIdx];
							float newDistance = -BiharmonicDistanceP2P_sorted(
												def->MVCWeights, 
												def->weightsSort, 
												pt, bd, 
												def->expansion, 
												def->precomputedDistances, 
												def->cuttingThreshold);

							if(assignedIdx < 0 || distance > newDistance)
							{
								assignedIdx = defbranchIdx;
								distance = newDistance;
							}
						}

						if(assignedIdx >= 0)
						*/
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

			/*
				if(group)
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
					// Deber�a haber al menos 1 nodo.
					// Lo empilamos para evaluar todo el segmento.
					//relevantNodes.push_back(jt->childs[childIdx]->nodes[0]);

					double thresh = -10;// bd->weightsCutThreshold;
				
					float bestDistance = 9999999;
					int bestNode = -1;

					float secondDistance = 99999099;
					int secondNode = -1;

					// Tengo que obtener la informaci�n correspondiente a este punto, para ir m�s r�pido.

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

				}
			}
			*/
		}
	}

	}

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
					pointsFromThisGroup.push_back(&pd);
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
		for(int row = 0; row < model.bind->BihDistances[surfaceIdx].size; row++)
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
		}
	}

	if(ascii)
		fclose(foutascii);
	else
		foutbin.close();
	
}