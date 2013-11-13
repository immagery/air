#ifndef AIR_RIG_H
#define AIR_RIG_H

#include <iostream>
#include <fstream>

#include <vector>

#include <DataStructures/skeleton.h>
#include <DataStructures/DefNode.h>
#include <DataStructures/Modelo.h>
#include <DataStructures/rig.h>
#include <DataStructures/AirSkinning.h>

#include <Eigen/Dense>

#define endChop_DEF 0.001
#define ratioExpansion_DEF 0.7

#define default_SMOOTHING_PASES 3
#define default_SMOOTHING_RATIO 0.25

#define default_SUBDIVISION_RATIO 0.25
#define default_EXPANSION 1

using namespace std;
using namespace Eigen;



// This clases define the dependency relationship between deformers,
class ControlGroup : public node
{
public:
	ControlGroup() {}
};

enum DefGroupType { DEF_NONE = 0, DEF_POINT, DEF_STICK, DEF_SURFACE, DEF_VOLUME};

class defGroupSerialization
{
public:
	defGroupSerialization(){}
	string sJointName;
};

class DefGroup : public node
{
public:
	DefGroup(int nodeId);
	DefGroup(int nodeId, joint* jt);

	vector<DefNode> deformers;
	joint* transformation;

	float subdivisionRatio;

	float expansion;
	int smoothingPasses;
	float smoothPropagationRatio;

	DefGroupType type; 

	// For computational pourposes.
	vector<DefGroup*> relatedGroups;

	defGroupSerialization* serializedData;

	bool saveToFile(FILE* fout);
	bool loadFromFile(FILE* fout);
};

class constraintSerialization
{
public:
	int parentId;
	int childId;
};

class Constraint : public node
{
public:
	Constraint()
	{
		weight = 1.0;
	}

	DefGroup* parent;
	DefGroup* child;
	int weight;

	constraintSerialization* sConstraint;

	// Serialization
	virtual bool saveToFile(FILE* fout);

	// How can a I load the references.
	virtual bool loadFromFile(FILE* fout);

};

class ControlGraph : public node
{
public:
	ControlGraph(){}

	// Serialization
	virtual bool saveToFile(FILE* fout)
	{ 
		fout = fout; // for delete warning 
		return true;
	}
	virtual bool loadFromFile(FILE* fout)
	{
		fout = fout; // for delete warning
		return true;
	}

};


class DefGraph : public node
{
public:
	DefGraph()
	{
		roots.clear();

		joints.clear();
		defGroups.clear();

		deformers.clear();
		relations.clear();

		defGroupsRef.clear();

		smoothingPasses = 3;
		smoothPropagationRatio = 0.25;
	}

	vector<DefGroup*> roots;

	vector<joint*> joints;
	vector<DefGroup*> defGroups;

	vector<DefNode*> deformers;
	vector<Constraint* > relations;

	map<unsigned int, DefGroup*> defGroupsRef;
	map<unsigned int, DefNode*> defNodesRef;

	int smoothingPasses;
	float smoothPropagationRatio;

	bool saveToFile(FILE* fout);
	bool loadFromFile(FILE* fout);
};


class airRigSerialization
{
public:
	string sModelName;
	vector<string> sSkeletonName;
};

// This class containst all the info and processes for
// build a perfect rig.
class AirRig : public rig
{
public:

	// Elements that computes the deformation of the model.
	DefGraph defRig;

	// Elements for control the rig
	ControlGraph controlRig;

	// The base model
	Modelo* model;

	// The skeletons used to compute skinning, or used as output
	vector<skeleton*> skeletons;

	//The deformer
	AirSkinning* skinning;

	// Parameters for computation
	double iniTwist; // To replace in the group parameters
	double finTwist; // To replace in the group parameters
	bool enableTwist; // To replace in the group parameters

	airRigSerialization* serializedData;

	// Constructors
	AirRig(int id); // default, without model bind
	AirRig(Modelo* model, int id);
	AirRig(Modelo* model, vector<skeleton*>& skts, int id);

	// Parameter inizialization
	void initParameters();

	// Destructor
	~AirRig();

	// updates the info of the deformation nodes
	// The data needs to be consistent, every DefGroup with a joint depending on the type.
	bool updateDefGroups();

	bool saveToFile(FILE* fout);
	bool loadFromFile(FILE* fout);

	// Load the rig defined in the file
	virtual bool loadRigging(string sFile);

	virtual bool bindRigToScene(Modelo& model, vector<skeleton*>& skeletons);

};


class PointConstraint : public Constraint
{
public:
	Vector3d offset;
	Vector3d value;

	PointConstraint() : Constraint()
	{
		offset = Vector3d(0,0,0);
		value = Vector3d(0,0,0);
	}

	// Serialization
	virtual bool saveToFile(FILE* fout)
	{ 
		fout = fout; // for delete warning
		return true;
	}
	virtual bool loadFromFile(FILE* fout)
	{ 
		fout = fout; // for delete warning
		return true;
	}

};

class OrientConstraint : public Constraint
{
public:
	Vector3d offset;
	Vector3d value;

	OrientConstraint() : Constraint()
	{
		offset = Vector3d(0,0,0);
		value = Vector3d(0,0,0);
	}

	// Serialization
	virtual bool saveToFile(FILE* fout)	
	{ 
		fout = fout; // for delete warning
		return true;
	}

	virtual bool loadFromFile(FILE* fout)	
	{ 
		fout = fout; // for delete warning
		return true;
	}
};

class ScaleConstraint : public Constraint
{
public:
	Vector3d offset;
	Vector3d value;

	ScaleConstraint() : Constraint()
	{
		offset = Vector3d(0,0,0);
		value = Vector3d(0,0,0);
	}

	// Serialization
	virtual bool saveToFile(FILE* fout)	
	{ 
		fout = fout; // for delete warning
		return true;
	}
	virtual bool loadFromFile(FILE* fout)	
	{ 
		fout = fout; // for delete warning
		return true;
	}
};

class ParentConstraint : public Constraint
{
public:
	OrientConstraint or;
	PointConstraint pos;

	ParentConstraint() : Constraint()
	{
	}

	// Serialization
	virtual bool saveToFile(FILE* fout)	
	{ 
		fout = fout; // for delete warning
		return true;
	}
	virtual bool loadFromFile(FILE* fout)	
	{ 
		fout = fout; // for delete warning
		return true;
	}
};

class HierarchyConstraint : public ParentConstraint
{
public:
	ScaleConstraint slc;

	HierarchyConstraint() : ParentConstraint()
	{
	}

	// Serialization
	virtual bool saveToFile(FILE* fout)	
	{ 
		fout = fout; // for delete warning
		return true;
	}
	virtual bool loadFromFile(FILE* fout)	
	{ 
		fout = fout; // for delete warning
		return true;
	}
};

//FUNCTIONS FOR PROCESSING DATA STRUCTURES

// Propagate a expansión value, depending of the child associated
bool propagateExpansion(DefGroup& gr, float parentValue, int childId, float childValue);

// Process just one skeleton adding the info to last loaded skeletons
bool processSkeleton(skeleton* skt, DefGraph& defRig);

// Propose deformation nodes depending on the joint linked
bool proposeDefNodesFromStick(DefGroup& group, vector<DefGroup*> relatedGroups );

// Propose a division in deformers from a given stick
int subdivideStick(Vector3d origen, Vector3d fin, int defGorupIdx, int childDefGorupIdx,
				   vector< DefNode >& nodePoints, float subdivisionRatio);

// Takes all the groups and constraints and
// build a relation tree for weights computation
void BuildGroupTree(DefGraph& graph);


#endif // AIR_RIG_H