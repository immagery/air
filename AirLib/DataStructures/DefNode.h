#ifndef DEFNODE_H
#define DEFNODE_H

#include "DataStructures.h"

#include <vector>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;
using namespace vcg;

class DefNode
{
public:
	DefNode()
	{
		boneId = -1;
		nodeId = -1;
		rootBoneId = -1;

		ratio = -1;
		expansion = 1;

		pos = Point3d(0,0,0);
		precomputedDistances = 0;
	}

	DefNode(int id)
	{
		boneId = id;
		nodeId = -1;
		rootBoneId = -1;

		ratio = -1;
		expansion = 1;

		pos = Point3d(0,0,0);
		precomputedDistances = 0;
	}

	DefNode(Point3d newPos, int id)
	{
		boneId = id;
		nodeId = -1;

		ratio = -1;
		expansion = 1;

		pos = newPos;
		precomputedDistances = 0;
	}

	DefNode(const DefNode& def_orig)
	{
		boneId = def_orig.boneId;
		nodeId = def_orig.nodeId;
		rootBoneId = def_orig.rootBoneId;

		ratio = def_orig.ratio;
		expansion = def_orig.expansion;

		pos = def_orig.pos;
		precomputedDistances = def_orig.precomputedDistances;
	}

	int boneId; // Who is my parent?
	int nodeId;
	int rootBoneId;

	float ratio; // position over the bone (for blending through the bone)
	float expansion; // influence expansion (for segmentation)

	// Aux variable that encapsulates precomputed part af distance calculus.
	double precomputedDistances;

	// For faster computation
	Point3d pos;
};

#endif // GRID3D_H
