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
}

void ComputationMgr::updateAllComputations(AirRig* rig)
{
	// By now we are reducing the process to just one binding.
	for(unsigned int defId = 0; defId< rig->defRig.deformers.size(); defId++)
    {
		if(rig->defRig.deformers[defId]->dirtyFlag)
			preprocessNodeCPU(rig->defRig.deformers[defId]);
	}

	// Updating Segmentation
	segmentModelFromDeformers(*model, bd, rig->defRig);

	// Update Smooth propagation
	propagateHierarchicalSkinning(*model, bd, rig->defRig);

	// Compute Secondary weights ... by now compute all the sec. weights
	computeSecondaryWeights(*model, bd, rig->defRig);
}