#include "AirSegmentation.h"

#include <Computation\Segmentation.h>
#include <Computation\mvc_interiorDistances.h>
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

void BuildGroupTree(DefGraph& graph)
{
	map<unsigned int, DefGroup*>& groups = graph.defGroupsRef;
	for(int grIdx = 0; grIdx < graph.defGroups.size(); grIdx++)
	{
		groups[graph.defGroups[grIdx]->nodeId] = graph.defGroups[grIdx];
		graph.defGroups[grIdx]->relatedGroups.clear();
	}

	map<int, bool> parented;
	for(int ctrIdx = 0; ctrIdx < graph.relations.size(); ctrIdx++)
	{
		// there are serveral types of constraint. Depending on what constraint is defined,
		// the computation tree is updated.
		graph.relations[ctrIdx]->parent->relatedGroups.push_back(graph.relations[ctrIdx]->child);
		parented[graph.defGroupsRef[graph.relations[ctrIdx]->child->nodeId]->nodeId] = true;
	}

	for(int grIdx = 0; grIdx < graph.defGroups.size(); grIdx++)
	{
		// If there is no parent from this joint it will be classified as root
		if(parented.count(graph.defGroups[grIdx]->nodeId) == 0)
		{
			graph.roots.push_back(graph.defGroups[grIdx]);
		}
	}
}

void updateAirSkinning(DefGraph& graph, Modelo& model)
{
	vector<int> traductionTable;
	map<int, DefNode*> nodeIds;

	// Creamos la tabla de traducción general.
	traductionTable.resize(graph.deformers.size());

    for(unsigned int j = 0; j< traductionTable.size(); j++)
        nodeIds[graph.deformers[j]->nodeId] = graph.deformers[j];

	binding* bd = model.bindings[0];

	// Updating deformer's info
	for(unsigned int defId = 0; defId< graph.deformers.size(); defId++)
    {
		// Only the dirty deformers
		if(graph.deformers[defId] -> dirtyFlag)
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
		}
	}

	// By now we are reducing the process to just one binding.

	// Updating Segmentation
	segmentModelFromDeformers(model, model.bindings[0], graph);

	// Update Smooth propagation
	propagateHierarchicalSkinning(model, model.bindings[0], graph);
	
}

void createTraductionTable(DefGroup* group, map<int, int>& traductionTable, int idNode, bool onlyRoot)
{
	    // Los nodos correspondientes al hueso jt
	for(unsigned int i = 0; i< group->deformers.size(); i++)
    {
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

void SmoothFromSegment(Modelo& m, binding* bd, DefGroup* group, int frontId)
{
    // Fronts creation using frontId
    vector< int > front;

    // Init grid for weights smoothing
	if(VERBOSE) printf("\n-- initCellWeightsSmoothing --\n");fflush(0);
    initSurfaceWeightsSmoothing(m, bd, front, frontId);

    // Smoothing (sin corte jerárquico)
	if(VERBOSE) printf("-- Smoothing -- \n");fflush(0);

	//int smoothingPasses = bd->smoothingPasses;
	//float realSmooth = bd->smoothPropagationRatio;

	int smoothingPasses = group->smoothingPasses;
	float realSmooth = group->smoothPropagationRatio;

    weightsSmoothing(m, bd, front, realSmooth, frontId, smoothingPasses);
	//weightsNoSmooting(m, bd, front, frontId);
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

	for(unsigned int i = 0; i< group.relatedGroups.size(); i++)
	{
		// Preparamos la traduccion para el grid.
		createTraductionTable(group.relatedGroups[i], traductionTable, group.relatedGroups[i]->nodeId);
        segmentationIds.push_back(group.relatedGroups[i]->nodeId);
	}


	// 2.b. Add also the father for fighting (TODEBUG IN TEST-TIME)
	createTraductionTable(&group, traductionTable, group.nodeId, true);
	//segmentationIds.push_back(jt->nodeId);

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

void propagateHierarchicalSkinning(Modelo& model, binding* bd, DefGraph& graph)
{
	clearOlderComputations(model);

	// 1. Establecemos el dominio
	if(VERBOSE) printf("1. Domain initzialization\n");fflush(0);
	initDomain(model, model.bindings[0], FIRST_ITERATION);

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

	for(unsigned int i = 0; i< graph.roots.size(); i++)
	{
        propagateHierarchicalSkinning( model,  bd, graph, *graph.roots[i]);
	}

    cleanZeroInfluences(bd);
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
		dirtyDeformers[deformers[deformerId]->nodeId] = deformers[deformerId]->dirtyFlag;
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

		def->dirtyFlag = false;
	}
}
