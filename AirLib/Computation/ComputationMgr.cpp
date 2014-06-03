#include "ComputationMgr.h"
#include "AirSegmentation.h"
#include "mvc_interiorDistances.h"


void ComputationMgr::setModelForComputations(Modelo* m)
{
	// Lo suyo sería hacer un preproceso para obtener piezas, grafos.
	// Y calcular las distancias de la manera que quiera.
	// Y tener así todo listo para los cálculos posteriores.

	// Por ahora suponemos que esta bien inicializado y construido
	model = m;
	bd = m->bind;

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
	vector<int> noFreeNodes;
	for(int ii = 0; ii < defNodesTotalSize; ii++)
	{
		DefNode* dn = rig->defRig.deformers[ii];
		
		if(!dn->addedToComputations) // It's not added yet to the computations
		{
			defNodeComputationReference[dn->nodeId] = firstPosToAdd+addedToComputationsCount;
			dn->addedToComputations = true;
			addedToComputationsCount++;
		}

		if(!dn->freeNode) // It's is a full computation node
		{
			noFreeNodes.push_back(ii);
		}
	}

	int newSize = MatrixWeights.cols() + addedToComputationsCount;
	assert(newSize >= noFreeNodes.size());

	assert(newSize == defNodesTotalSize && newSize == defNodeComputationReference.size());
	// Resize of temp computation structures if it is necesary.
	if(MatrixWeights.cols() < defNodesTotalSize)
	{
		// Matrix weights for comutations
		MatrixWeights.resize(model->vn(), newSize);

		// Init subDistances
		distancesTemp.resize(model->vn(), newSize);
	}

	if(VERBOSE_PROCESS)
	{
		printf("\n[updateAllComputations] - MEMORY ALLOCATION:\n");
		printf("1. Computation matrices: %d row x %d cols\n", MatrixWeights.rows(), MatrixWeights.cols());
		printf("2. Added to the computations: %d nodes\n", addedToComputationsCount);
		printf("3. Total nodes allocated: %d nodes\n", defNodesTotalSize);
	}

	//Compute per node data, with optimized code.
	computeNodesOptimized(rig->defRig, *model, MatrixWeights, distancesTemp, defNodeComputationReference);

	clock_t fin = clock();
	
	if(VERBOSE_PROCESS)
	{
		printf("\nProceso en: %fms\n", ((double)(fin-ini)));
	}
}