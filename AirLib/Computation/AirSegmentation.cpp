#include "AirSegmentation.h"

//#include <Computation\Segmentation.h>
#include <Computation\mvc_interiorDistances.h>
#include <Computation/BiharmonicDistances.h>

#include <DataStructures\InteriorDistancesData.h>

#include <DataStructures/Scene.h>

// Esta función debería actualizar los datos y relanzar los cálculos necesarios.
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
				// Este caso no debería darse porque las estructuras deberían estar mantenidas correctamente.
			}

			// Volver a proponer los nodos, sin perder información de usuario.
		}

		// Marcamos el joint como hecho, así podemos llevar un control de la coherencia de los datos.
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

void updateAirSkinning(DefGraph& graph, Modelo& model)
{
	vector<int> traductionTable;
	map<int, DefNode*> nodeIds;

	// Creamos la tabla de traducción general.
	traductionTable.resize(graph.deformers.size());

    for(unsigned int j = 0; j< traductionTable.size(); j++)
        nodeIds[graph.deformers[j]->nodeId] = graph.deformers[j];

	binding* bd = model.bind;

	// Updating deformer's info
	for(unsigned int defId = 0; defId< graph.deformers.size(); defId++)
    {
		// Only the dirty deformers
		if(graph.deformers[defId]->dirtyFlag)
		{
			// Mean value coordinates computation.
			mvcAllBindings(graph.deformers[defId]->pos, 
						   graph.deformers[defId]->MVCWeights,
						   model);
			
			// TOREMOVE: it's only for select adaptative threshold
			graph.deformers[defId]->cuttingThreshold = 1;

			// TOOPTIMIZE: Weights sort, now with bubble approach
			doubleArrangeElements_wS_fast(graph.deformers[defId]->MVCWeights, 
										  graph.deformers[defId]->weightsSort, 
										  graph.deformers[defId]->cuttingThreshold);

			// By now just one embedding
			graph.deformers[defId]->precomputedDistances = PrecomputeDistancesSingular_sorted(graph.deformers[defId]->MVCWeights, 
																							  graph.deformers[defId]->weightsSort, 
																							  bd->BihDistances, 
																							  graph.deformers[defId]->cuttingThreshold);
			graph.deformers[defId]->dirtyFlag = false;
			graph.deformers[defId]->segmentationDirtyFlag = true;
		}
	}

	// By now we are reducing the process to just one binding.

	// Updating Segmentation
	segmentModelFromDeformers(model, model.bind, graph);

	// Update Smooth propagation
	propagateHierarchicalSkinning(model, model.bind, graph);

	// Compute Secondary weights ... by now compute all the sec. weights
	computeSecondaryWeights(model, model.bind, graph);
	
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

    // Descendemos por la jerarquía marcando los nodos de los huesos hijos.
    for(unsigned int jtChild = 0; jtChild < group->relatedGroups.size(); jtChild++)
    {
		createTraductionTable(group->relatedGroups[jtChild], traductionTable, idNode);
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

void SmoothFromSegment(Modelo& m, binding* bd, DefGroup* group, int frontId)
{
    // Fronts creation using frontId
    vector< int > front(0);

    // Init model for weights smoothing
	if(VERBOSE) printf("\n-- initCellWeightsSmoothing --\n");fflush(0);
    initSurfaceWeightsSmoothing(m, bd, front, frontId);

    // Smoothing (sin corte jerárquico)
	if(VERBOSE) printf("-- Smoothing -- \n");fflush(0);

	// get the global or local smoothing value
	int smoothingPasses = group->smoothingPasses;
	float realSmooth = group->smoothPropagationRatio;

    weightsSmoothing(m, bd, front, realSmooth, frontId, smoothingPasses);
	//weightsNoSmooting(m, bd, front, frontId);
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
            // Para cada elemento del frente realizamos la expansión.
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

				// Distancia entre puntos... cuando tenga biharmonic distances ya estará calculado.
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
				// Para cada elemento del frente realizamos la expansión.
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
					printf("Como puede ser que no haya ningun peso?.\n");
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

	// 5. Normalización de pesos basados en el dominio
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

bool ComputeEmbeddingWithBD(Modelo& model, bool withPatches)
{
	binding* bds = model.bind;

	// Computation indices
	vector<int> indices;
	indices.resize(bds->pointData.size());
	for(int idx = 0; idx < bds->pointData.size(); idx++)
	{
		//indices.push_back(idx);
		//indices.push_back(bds[bind]->pointData[idx].node->id);
		indices[idx] = bds->pointData[idx].node->id;
	}

	printf("\n\nCalculo de A para embeding: [%d puntos]\n", bds->pointData.size());
	fflush(0);
	bindingBD( model, bds, indices, bds->BihDistances, withPatches);

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

	for(int bind = 0; bind < m.bind->surfaces.size(); bind++)
	{
		int pointsSize = 0;
		finbin.read( (char*) &pointsSize,  sizeof(int) );

		// Debe corresponder el numero de puntos
		if(pointsSize != m.bind->surfaces[bind].nodes.size())
			return false;

		m.bind->BihDistances.resize(pointsSize);
		for(int row = 0; row < m.bind->BihDistances.size; row++)
		{
			for(int col = row; col < m.bind->BihDistances.size; col++)
			{
				double value = 0;
				finbin.read( (char*) &value,  sizeof(double) );
				m.bind->BihDistances.set(row,col,value);
			}
		}
	}
	
	finbin.close();
	return true;
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

	// 3. Smooth entre hijos cortando según el dominio
	if(VERBOSE){printf("3. Segments smoothing\n");fflush(0);}

	for(unsigned int i = 0; i< segmentationIds.size(); i++)
	{
		SmoothFromSegment(model, bd, graph.defGroupsRef[segmentationIds[i]], segmentationIds[i]);
	}

	// 5. Normalización de pesos basados en el dominio
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
				if(distance > newDistance || newSegmentId == -1)
				{
					distance = newDistance;
					newSegmentId = def->nodeId;
				}
			}

			pd.segmentId = newSegmentId;
			pd.segmentDistance = distance;
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
				if(newDistance < pd.segmentDistance)
				{
					pd.segmentDistance = newDistance;
					pd.segmentId = def->nodeId;
				}
			}
		}

		def->segmentationDirtyFlag = false;
	}
}

int indexOfNode(int nodeId, vector<DefNode>& nodes)
{
	for(int i = 0; i< nodes.size(); i++)
	{
		if(nodes[i].nodeId == nodeId) return i;
	}

	return -1;
}

void computeSecondaryWeights(Modelo& model, binding* bd, DefGraph& graph)
{
    // No hay ningun binding
    if(!bd ) return;

	vector< DefNode >& points = bd->intPoints;
	vector< vector<double> >& weights = bd->embeddedPoints;
	vector< vector<int> >& sortedWeights = bd->weightsSort;
	vector< vector<weight> >& weightsClean = bd->weightsFiltered; 
		
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

			dp.secondInfluences[infl].resize(group->relatedGroups.size(), 0.0);
			vector<bool> assigned;
			assigned.resize(group->relatedGroups.size(), false);

			// Si tiene hijos o realmente encuentra el deformador.... hay que ver segmentId, que tal.
			int idxNodeAsignedAux = indexOfNode(dp.segmentId,group->deformers);

			if(idxNodeAsignedAux<0)
			{
				bool found = false;
				for(int childIdx = 0; childIdx < group->relatedGroups.size() && !found; childIdx++)
				{
					found |= (indexOfNode(dp.segmentId,group->relatedGroups[childIdx]->deformers)>= 0);
					if(found)
					{
						dp.secondInfluences[infl][childIdx] = 1.0;
						break;
					}
				}
				continue;
			}
			
			// El Hijo que ya esta asignado
			DefNode* asignedNode = &group->deformers[idxNodeAsignedAux];
			for(int childIdx = 0; childIdx < group->relatedGroups.size(); childIdx++)
			{
				if(assigned[childIdx]) continue;

				if(group->relatedGroups[childIdx]->nodeId == asignedNode->childBoneId)
				{
					dp.secondInfluences[infl][childIdx] = asignedNode->ratio;
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
						dp.secondInfluences[infl][childIdx] = defs[nodeIdChildSegment]->ratio;
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

				}
			}
			*/
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
		printf("Guardado embedding en fichero de texto: %s\n", binFile);
	}

	if(ascii)
		fprintf(foutascii,"%d\n", model.bind->pointData.size());
	else
	{
		int ss = model.bind->pointData.size();
		foutbin.write((const char*) &ss, sizeof(int));
	}

	for(int row = 0; row < model.bind->BihDistances.size; row++)
	{
		for(int col = row; col < model.bind->BihDistances.size; col++)
		{
			if(ascii)
				fprintf(foutascii,"%f\n", model.bind->BihDistances.get(row,col));
			else
			{
				double val = model.bind->BihDistances.get(row,col);
				foutbin.write((const char*) &val, sizeof(double));
			}
		}
	}

	if(ascii)
		fclose(foutascii);
	else
		foutbin.close();
	
}