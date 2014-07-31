#include "ComputationMgr.h"
#include "AirSegmentation.h"
#include "mvc_interiorDistances.h"


void ComputationMgr::setModelForComputations(Modelo* m, int in_surfaceIdx)
{
	// Lo suyo sería hacer un preproceso para obtener piezas, grafos.
	// Y calcular las distancias de la manera que quiera.
	// Y tener así todo listo para los cálculos posteriores.

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

	bool bCreationComputationNeeded = false;
	bool bSegmentationComputationNeeded = false;
	bool bExpansionComputationNeeded = false;
	bool bSmoothingComputationNeeded = false;

	int insideNodes = 0;
	int dirtyCount = 0;
	// Preparing memory for allocate all the computations
	int addedToComputationsCount = 0;

	map<unsigned int, DefGroup*>::iterator it = rig->defRig.defGroupsRef.begin();
	while (it != rig->defRig.defGroupsRef.end())
	{
		int defSize = it->second->deformers.size();
		for (int ii = 0; ii < defSize; ii++)
		{
			DefNode* dn = &it->second->deformers[ii];
			defNodeIsInside[dn->nodeId] = false;

			// debug: Test if the cell is in the grid
			Vector3i nodeCell = model->grid->cellId(dn->pos);
			if (nodeCell.x() >= 0 && nodeCell.y() >= 0 && nodeCell.z() >= 0 &&
				nodeCell.x() < model->grid->dimensions.x() && nodeCell.y() < model->grid->dimensions.y() && nodeCell.z() < model->grid->dimensions.z())
			{
				if (model->grid->isContained(nodeCell, surfaceIdx))
				{
					defNodeIsInside[dn->nodeId] = true;
					insideNodes++;
				}
			}

			if (defNodeComputationReference.find(dn->nodeId) == defNodeComputationReference.end())
			{
				// This node is not in the computations
				defNodeComputationReference[dn->nodeId] = firstPosToAdd + addedToComputationsCount;
				dn->addedToComputations = true;
				addedToComputationsCount++;
			}

			defNodeDirtyBit[dn->nodeId] = dn->segmentationDirtyFlag || dn->expansionDirtyFlag || dn->smoothDirtyFlag || !defNodeIsInside[dn->nodeId];

			// Si no estan no procesamos nada
			bSegmentationComputationNeeded |= dn->segmentationDirtyFlag && defNodeIsInside[dn->nodeId];
			bExpansionComputationNeeded |= dn->expansionDirtyFlag && defNodeIsInside[dn->nodeId];

			if (defNodeDirtyBit[dn->nodeId]) dirtyCount++;

			bSmoothingComputationNeeded |= (it->second->dirtySmooth && defNodeIsInside[dn->nodeId]) || bSegmentationComputationNeeded || bExpansionComputationNeeded;
		}
		it++;
	}

	//printf("Nodos sucios que estaria bien actualizar: %d\n", dirtyCount);

	int newSize = MatrixWeights.cols() + addedToComputationsCount;
	assert(newSize == defNodesTotalSize && newSize == defNodeComputationReference.size());

	// Resize of temp computation structures if it is necesary.
	if (MatrixWeights.cols() < defNodesTotalSize)
	{
		// Matrix weights for comutations
		MatrixXf tempDistances = MatrixWeights;
		MatrixWeights.resize(model->bind->surfaces[surfaceIdx].nodes.size(), newSize);
		//MatrixWeights.fill(0);
		for (int tempRow = 0; tempRow < tempDistances.rows(); tempRow++)
		{
			MatrixWeights.row(tempRow).segment(0, tempDistances.cols()) = tempDistances.row(tempRow);
		}

		// Init subDistances
		tempDistances = distancesTemp;
		distancesTemp.resize(model->bind->surfaces[surfaceIdx].nodes.size(), newSize);
		//distancesTemp.fill(999999999);
		for (int tempRow = 0; tempRow < tempDistances.rows(); tempRow++)
		{
			distancesTemp.row(tempRow).segment(0, tempDistances.cols()) = tempDistances.row(tempRow);
		}

		// Final computed distances matrix
		tempDistances = computedDistances;
		computedDistances.resize(model->bind->surfaces[surfaceIdx].nodes.size(), newSize);
		//computedDistances.fill(999999999);
		for (int tempRow = 0; tempRow < tempDistances.rows(); tempRow++)
		{
			computedDistances.row(tempRow).segment(0, tempDistances.cols()) = tempDistances.row(tempRow);
		}
	}

	if (VERBOSE_PROCESS)
	{
		printf("\n[updateAllComputations] - MEMORY ALLOCATION:\n");
		printf("1. Computation matrices: %d row x %d cols\n", MatrixWeights.rows(), MatrixWeights.cols());
		printf("2. Added to the computations: %d nodes\n", addedToComputationsCount);
		printf("3. Total nodes allocated: %d nodes\n", defNodesTotalSize);
		printf("4. Nodes inside this surface: %d\n", insideNodes);
	}

	//Compute per node data, with optimized code.

	if (bSegmentationComputationNeeded || bCreationComputationNeeded)
	{
		computeNodesOptimized(rig->defRig, *model, MatrixWeights,
								distancesTemp, computedDistances, precomputedDistances,
								defNodeComputationReference, defNodeDirtyBit,
								defNodeIsInside, surfaceIdx);
	}
	else if (bExpansionComputationNeeded)
	{
		computeNodesOptimized_justExpansion(rig->defRig, *model, MatrixWeights,
											distancesTemp, computedDistances, precomputedDistances,
											defNodeComputationReference, defNodeDirtyBit,
											defNodeIsInside, surfaceIdx);
	}

	// Testing calcular solo smoothing
	if (bSmoothingComputationNeeded)
	{
		propagateHierarchicalSkinningOpt(*model, model->bind, rig->defRig, surfaceIdx);
	}

	// Compute Secondary weights ... by now compute all the sec. weights
	//computeSecondaryWeightsOpt(model, model.bind, graph, distancesTemp, defNodeRef, computedDistances, surfaceIdx, false);

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
	if (VERBOSE_PROCESS)
	{
		printf("\n\n---------- TIEMPOS GENERALES -------------------------- \n");
		printf("PIEZA: %d\n", surfaceIdx);
		
		//printf("A. Ver que hay de nuevo:  %f\n", ((double)(fin00 - ini00))/CLOCKS_PER_SEC*1000);
		//printf("   >>> Bucle:  %f, mean: %f\n", ((double)(fin00_00 - ini00_00)) / CLOCKS_PER_SEC * 1000, ((double)(fin00_00 - ini00_00)) / CLOCKS_PER_SEC * 1000 / defNodesTotalSize);
		//printf("		>>> Bucle in0 :  %f\n", ((double)(fin00_00_00 - ini00_00_00)) / CLOCKS_PER_SEC * 1000 / defNodesTotalSize);
		//printf("		>>> Bucle in1 :  %f\n", ((double)(fin00_00_01 - ini00_00_01)) / CLOCKS_PER_SEC * 1000 / defNodesTotalSize);
		//printf("		>>> Bucle in2 :  %f\n", ((double)(fin00_00_02 - ini00_00_02)) / CLOCKS_PER_SEC * 1000 / defNodesTotalSize);
		//printf("		>>> Bucle in3 :  %f\n", ((double)(fin00_00_03 - ini00_00_03)) / CLOCKS_PER_SEC * 1000 / defNodesTotalSize);
		//printf("   >>> 2o Bucle:  %f, mean: %f\n", ((double)(fin00_01 - ini00_01)) / CLOCKS_PER_SEC * 1000, ((double)(fin00_01 - ini00_01)) / CLOCKS_PER_SEC * 1000 / rig->defRig.defGroupsRef.size());
		//printf("B. Inicializar datos acordemente:  %f\n", ((double)(fin01 - ini01)) / CLOCKS_PER_SEC * 1000);
		//printf("C. El calculo propiamente: %f\n", ((double)(fin02 - ini02)) / CLOCKS_PER_SEC * 1000);
		//printf("D. Limpieza: %f\n", ((double)(fin03 - ini03)) / CLOCKS_PER_SEC * 1000);
		printf("\n TOTAL: %f\n", ((double)(fin - ini)) / CLOCKS_PER_SEC * 1000);
		printf("------------------------------------------------------------");
	}

}