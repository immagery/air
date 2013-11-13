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
	~AirSkinning();

	virtual void loadBindingForModel(Modelo *m, string path, const vector< skeleton* >& skeletons);
	virtual void saveBindingToFile (string path);

	// For update linking: initialization
	void cacheSkinning();
	void getDeformerRestPositions();

	// Compute deformations over the deformed model taking the original model,
	// and the skinning cached
	virtual void computeDeformations();
	virtual void computeDeformationsWithSW();

	// Reference to rigg deformers
	map< int, joint* > deformersRestPosition;
	map< int, joint* > deformersPosition;

	// Parent rig
	AirRig* rig;
};

#endif // AIR_SKINNING_H