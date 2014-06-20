#ifndef COMPUTATION_MGR_H
#define COMPUTATION_MGR_H

#include <Computation\AirSegmentation.h>
#include <DataStructures\AirRig.h>
#include <queue>

using namespace std;

enum computationsState { ST_CREATED = 0, ST_INIT, ST_UPDATED, ST_NOTVALID };

// Handles computation, in CPU or GPU, 
// depending on the platform
class ComputationMgr
{
public:
	// Constructor
	ComputationMgr()
	{
		model = NULL;
		bd = NULL;
		state = ST_CREATED;
		defNodeComputationReference.clear();

		surfaceIdx = 0;
	}

	// Init data for computations
	void setModelForComputations(Modelo* m, int surfaceIdx = 0);

	// Deprecated functions
	void preprocessNodeCPU(DefNode* node);

	// Update computations
	void updateAllComputations();
	
	// State of the computations
	computationsState state;

	// Model to do computations
	Modelo* model;

	// Wich surface from the mode
	int surfaceIdx;

	// binding of the model
	binding* bd;

	// AirRig
	AirRig* rig;

	// Runtime temporal sub-computations
	MatrixXf MatrixWeights;
	MatrixXf distancesTemp;

	// queue of work to do
	vector<DefNode*> defNodes;

	// queue of segmentation to do
	map<int, DefNode*> defNodeReferences;

	// Position of eachNode in mem for computation
	map<int, int> defNodeComputationReference;

	// how many defNodes are in the computations
	int defNodesCount;

};

#endif // SKELETON_H