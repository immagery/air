#ifndef COMPUTATION_MGR_H
#define COMPUTATION_MGR_H

#include <DataStructures\AirRig.h>
#include <queue>

using namespace std;

// Handles computation, in CPU or GPU, 
// depending on the platform
class ComputationMgr
{
public:
	ComputationMgr()
	{
		model = NULL;
		bd = NULL;
	}

	void setModelForComputations(Modelo* m);

	void preprocessNodeCPU(DefNode* node);

	void updateAllComputations(AirRig* rig);
	
	// Model to do computations
	Modelo* model;

	// binding of the model
	binding* bd;

	// queue of work to do
	queue<DefNode*> preprocessNodes;

	// queue of segmentation to do
	queue<DefNode*> segmentationNodes;
};

#endif // SKELETON_H