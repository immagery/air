#ifndef AIR_SKINNING_H
#define AIR_SKINNING_H

#include <DataStructures/Skinning.h>
#include <Computation\BulgeDeformer.h>
#include <vector>
using namespace std;

class AirRig;
class AirSkinning : public Skinning
{
public:
	AirSkinning() : Skinning()  { useSecondaryWeights = false; }
	~AirSkinning();

	virtual void loadBindingForModel(Modelo *m, AirRig* rig);
	//virtual void saveBindingToFile (string path);

	// For update linking: initialization
	void cacheSkinning();
	void getDeformerRestPositions(AirRig* rig);

	// Compute deformations over the deformed model taking the original model,
	// and the skinning cached
	virtual void computeDeformations(AirRig* rig);
	virtual void computeDeformationsWithSW(AirRig* rig);

	virtual void resetDeformations();

	// Reference to rigg deformers
	map< int, joint* > deformersRestPosition;
	map< int, joint* > deformersPosition;

	vector<BulgeDeformer> bulge;
};

void saveAirBinding(binding* bd, string fileName);
void loadAirBinding(binding* bd, string fileName);

#endif // AIR_SKINNING_H