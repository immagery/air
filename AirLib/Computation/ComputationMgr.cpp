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

	int defNodesSize = rig->defRig.deformers.size();
	int firstPosToAdd = MatrixWeights.cols();

	// Preparar memoria para actualizar nodos.
	int addedToComputationsCount = 0;
	for(int ii = 0; ii < defNodesSize; ii++)
	{
		DefNode* dn = rig->defRig.deformers[ii];
				
		if(!dn->addedToComputations)
		{
			defNodeComputationReference[dn->nodeId] = firstPosToAdd+ii;
			dn->addedToComputations = true;
			addedToComputationsCount++;
		}
	}

	int newSize = MatrixWeights.cols() + addedToComputationsCount;
	assert(newSize == defNodesSize);

	if(MatrixWeights.cols() != defNodesSize)
	{
		// Matrix weights for comutations
		MatrixWeights.resize(model->vn(), newSize);

		// Init subDistances
		distancesTemp.resize(model->vn(), newSize);
	}

	//printf("Anadir al calculo %d nodos\n", addedToComputationsCount);
	computeNodesOptimized(rig->defRig, *model, MatrixWeights, distancesTemp, defNodeComputationReference);

	clock_t fin = clock();

	printf("\nProceso en: %fms\n", ((double)(fin-ini)));

	/*
	if(state == ST_CREATED)
	{
		// Do computations
		computeNodesOptimized(rig->defRig, *model, MatrixWeights, distancesTemp);

		state = ST_INIT;
	}
	else if(state == ST_INIT || state == ST_UPDATED)
	{
		state == ST_NOTVALID;

		computeNodesOptimized(rig->defRig, *model, MatrixWeights, distancesTemp);
	}
	*/
}