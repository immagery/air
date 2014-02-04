#ifndef BULGE_DEFORMER_H
#define BULGE_DEFORMER_H

#include <vector>

#include <DataStructures\node.h>

using namespace std;

class BulgeGroup
{
public:
	// The vertex influeced in this Bulge Group
	vector<int> vertexGroup;

	// Which deformers play with this group
	vector<int> deformerIds;
};

class Geometry;
class binding;
class AirRig;

class BulgeDeformer : public node
{
public:

	// Funciones
	BulgeDeformer();
	void applyDeformation(Geometry* m, binding* b, AirRig* rig);

	// Datos
	vector<BulgeGroup*> groups;

	bool enabled;
};

#endif // BULGE_DEFORMER_H