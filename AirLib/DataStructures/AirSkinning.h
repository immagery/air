#ifndef AIR_SKINNING_H
#define AIR_SKINNING_H

#include <DataStructures/Skinning.h>
#include <vector>
using namespace std;

class AirRig;
class AirSkinning : public Skinning
{
public:
	AirSkinning() : Skinning() { useSecondaryWeights = false;  rig = NULL; }
	AirSkinning(AirRig* _rig) : Skinning() { useSecondaryWeights = false; rig = _rig; }

	// For update linking: initialization
	void cacheSkinning();
	void getDeformerRestPositions();

	// Compute deformations over the deformed model taking the original model,
	// and the skinning cached
	void computeDeformations();
	void computeDeformationsWithSW();

	bool useSecondaryWeights;
	vector< vector<weight> > weights;
	map< int, joint > deformersRestPosition;

	AirRig* rig;
};

#endif // AIR_SKINNING_H