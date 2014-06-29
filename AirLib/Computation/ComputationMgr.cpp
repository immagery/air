#include "ComputationMgr.h"
#include "AirSegmentation.h"
#include "mvc_interiorDistances.h"


void ComputationMgr::setModelForComputations(Modelo* m, int in_surfaceIdx)
{
	// Lo suyo ser�a hacer un preproceso para obtener piezas, grafos.
	// Y calcular las distancias de la manera que quiera.
	// Y tener as� todo listo para los c�lculos posteriores.

	// Por ahora suponemos que esta bien inicializado y construido
	model = m;
	bd = m->bind;

	surfaceIdx = in_surfaceIdx;

}

void ComputationMgr::preprocessNodeCPU(DefNode* node)
{
/*
	// Mean value coordinates computation.
	mvcAllBindings(node->pos, 
				   node->MVCWeights,
				   *model);
			
	// TOREMOVE: it's only for select adaptative threshold
	node->cuttingThreshold = 1;

	// TOOPTIMIZE: Weights sort, now with bubble approach
	doubleArrangeElements_wS_fast(node->MVCWeights, 
								  node->weightsSort, 
								  node->cuttingThreshold);

	//TODEBUG: BihDistances as a matrix
	// By now just one embedding
	node->precomputedDistances = PrecomputeDistancesSingular_sorted(node->MVCWeights, 
																	node->weightsSort, 
																	bd->BihDistances[0], 
																	node->cuttingThreshold);
	node->dirtyFlag = false;
	node->segmentationDirtyFlag = true;
*/
}

void ComputationMgr::updateAllComputations()
{
	clock_t ini = clock();

	int defNodesTotalSize = rig->defRig.deformers.size();
	int firstPosToAdd = MatrixWeights.cols();

	// Preparing memory for allocate all the computations
	int addedToComputationsCount = 0;
	for(int ii = 0; ii < defNodesTotalSize; ii++)
	{
		DefNode* dn = rig->defRig.deformers[ii];
		if(defNodeComputationReference.find(dn->nodeId) == defNodeComputationReference.end())
		{
			// This node is not in the computations
			defNodeComputationReference[dn->nodeId] = firstPosToAdd+addedToComputationsCount;
			dn->addedToComputations = true;
			addedToComputationsCount++;
		}

		defNodeDirtyBit[dn->nodeId] = dn->segmentationDirtyFlag;
	}

	int newSize = MatrixWeights.cols() + addedToComputationsCount;
	assert(newSize == defNodesTotalSize && newSize == defNodeComputationReference.size());

	// Resize of temp computation structures if it is necesary.
	if(MatrixWeights.cols() < defNodesTotalSize)
	{
		// Matrix weights for comutations
		MatrixWeights.resize(model->bind->surfaces[surfaceIdx].nodes.size(), newSize);

		// Init subDistances
		distancesTemp.resize(model->bind->surfaces[surfaceIdx].nodes.size(), newSize);
	}

	if(VERBOSE_PROCESS)
	{
		printf("\n[updateAllComputations] - MEMORY ALLOCATION:\n");
		printf("1. Computation matrices: %d row x %d cols\n", MatrixWeights.rows(), MatrixWeights.cols());
		printf("2. Added to the computations: %d nodes\n", addedToComputationsCount);
		printf("3. Total nodes allocated: %d nodes\n", defNodesTotalSize);
	}

	//Compute per node data, with optimized code.
	computeNodesOptimized(rig->defRig, *model, MatrixWeights, 
						  distancesTemp, precomputedDistances, 
						  defNodeComputationReference, defNodeDirtyBit, surfaceIdx);

	/*
	//auto ini5 = high_resolution_clock::now();
	// Update Smooth propagation
	propagateHierarchicalSkinningOpt(model, model->bind, rig->defRig);	
	//auto fin5 = high_resolution_clock::now();
		
	//auto ini6 = high_resolution_clock::now();
	// Compute Secondary weights ... by now compute all the sec. weights
	computeSecondaryWeightsOpt(*model, model->bind, rig->defRig, distancesTemp, 
								defNodeComputationReference, distancesTemp, false);
				*/

	clock_t fin = clock();
	
	//if(VERBOSE_PROCESS)
	//{
		printf("\n[Surf->%d] Update Computations: %fms\n", surfaceIdx, ((double)(fin-ini)));
	//}
}