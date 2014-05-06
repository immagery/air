#ifndef AIR_RIG_H
#define AIR_RIG_H

#include <iostream>
#include <fstream>

#include <vector>
#include <queue>

#include <DataStructures/skeleton.h>
#include <DataStructures/DefNode.h>
#include <DataStructures/Modelo.h>
#include <DataStructures/rig.h>
#include <DataStructures/AirSkinning.h>

#include <Eigen/Dense>

#define endChop_DEF 0.001
#define ratioExpansion_DEF 1.0

#define default_SMOOTHING_PASES 3
#define default_SMOOTHING_RATIO 0.25

#define default_SUBDIVISION_RATIO 0.25
#define default_EXPANSION 1

#define default_INI_TWIST 0.0
#define default_END_TWIST 1.0

using namespace std;
using namespace Eigen;

enum riggingMode {MODE_RIG = 0, MODE_CREATE, MODE_ANIM, MODE_TEST};

// This clases define the dependency relationship between deformers,
class ControlGroup : public node
{
public:
	ControlGroup() {}
};

enum DefGroupType { DEF_NONE = 0, DEF_POINT, DEF_STICK, DEF_SURFACE, DEF_VOLUME};

class airRigSerialization
{
public:
	string sModelName;
	vector<string> sSkeletonName;

	map<int, int> defNodesRelation;
	map<int, int> defGroupsRelation;
};

class defGroupSerialization
{
public:
	defGroupSerialization(){}

	string sJointName;
};

class DefGroup : public object
{
public:
	DefGroup(int nodeId);
	DefGroup(int nodeId, joint* jt);

	vector<DefNode> deformers;
	joint* transformation;

	joint* rigTransform;

	float subdivisionRatio;

	float expansion;
	int smoothingPasses;
	float smoothPropagationRatio;
	bool localSmooth; 

	float iniTwist;
	float finTwist;
	bool enableTwist;
	bool smoothTwist;

	int parentType;

	bool bulgeEffect;

	bool dirtyCreation;
	bool dirtyTransformation;
	bool dirtySegmentation;

	DefGroupType type; 

	// For computational pourposes.
	vector<DefGroup*> dependentGroups;
	vector<DefGroup*> relatedGroups;

	defGroupSerialization* serializedData;

	bool saveToFile(FILE* fout);
	bool loadFromFile(ifstream& in, airRigSerialization* sData);

	virtual void setRotation(double rx, double ry, double rz, bool radians);
	
	void addTranslation(double tx, double ty, double tz);
	void setTranslation(double tx, double ty, double tz, bool local);
	void addRotation(double rx, double ry, double rz);
	void addRotation(Eigen::Quaternion<double> q, bool local = true);
	void setRotation(Eigen::Quaternion<double> q, bool local = true);

	Quaterniond getRotation(bool local = true);
	Vector3d getTranslation(bool local = true);

	Vector3d getRestTranslation(bool local = true);
	Quaterniond getRestRotation(bool local = true);

	bool dirtyByTransformation(bool alsoFather, bool hierarchically = true);
	bool dirtyBySegmentation();
	bool dirtyBySmoothing();

	void computeRestPos(DefGroup* dg, DefGroup* father = NULL);
	void computeWorldPos(DefGroup* dg, DefGroup* father = NULL);

	void restorePoses(DefGroup* dg, DefGroup* father = NULL);

	virtual bool propagateDirtyness()
    {
        dirtyFlag = true;

		for(int i = 0; i< relatedGroups.size(); i++)
			relatedGroups[i]->propagateDirtyness();

        return true;
    }

    virtual bool update()
    {
        if(!dirtyFlag)
            return true;
        else
		{
			// Se supone que el padre esta bien
			// Transmitimos el update a los hijos a
			// partir de aqui.

			if(dependentGroups.size() > 0)
				computeWorldPosRec(this, dependentGroups[0]);
			else
				computeWorldPosRec(this, NULL);

			/*for(int i = 0; i< relatedGroups.size(); i++)
			{
				relatedGroups[i]->update();
			}
			*/
			dirtyFlag = false;
            return true;
		}
    }

	virtual bool select(bool bToogle, unsigned int id);
	virtual bool selectRec(bool bToogle);

	void computeWorldPosRec(DefGroup* dg, DefGroup* fatherDg);
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
	virtual bool loadFromFile(ifstream& in);

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
	virtual bool loadFromFile(ifstream& in)
	{
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

		iam = DEFGRAPH_NODE;
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
	bool loadFromFile(ifstream& in, airRigSerialization* sData);

};

// This class containst all the info and processes for
// build a perfect rig.
class AirRig : public Rig
{
public:

	// Elements that computes the deformation of the model.
	DefGraph defRig;

	// Elements for control the rig
	ControlGraph controlRig;

	//The deformer
	AirSkinning* airSkin;

	// Parameters for computation
	//Default twist values;
	double iniTwist;
	double finTwist;
	bool enableTwist;

	// Default smooth value;
	int defaultSmoothPasses;

	// Toogle from rigg to animation mode
	static riggingMode mode;

	airRigSerialization* serializedData;

	// Constructors
	AirRig(int id); // default, without model bind
	//AirRig(Modelo* model, int id);
	//AirRig(Modelo* model, vector<skeleton*>& skts, int id);

	// Parameter inizialization
	void initParameters();

	// Destructor
	~AirRig();

	// updates the info of the deformation nodes
	// The data needs to be consistent, every DefGroup with a joint depending on the type.
	bool updateDefGroups();

	bool saveToFile(FILE* fout);
	bool loadFromFile(ifstream& in);

	// Load the rig defined in the file
	virtual bool loadRigging(string sFile);

	virtual bool bindLoadedRigToScene(Modelo* model, vector<skeleton*>& skeletons);
	virtual bool bindRigToModelandSkeleton(Modelo* in_model, vector<skeleton*>& in_skeletons, float subdivision);

	// This function just binds the model to the rigg and get all the mesh precomputations needed to work
	void initRigWithModel(Modelo* in_model);
	
	// Builds a deformer nodes configuration depending on the type
	void BuildGroup(DefGroup* def);

	// Adds more refinement with more nodes... that could be computed an optimized way.
	void RefineGroup(DefGroup* def);

	// recoge los nodos que deben re-preprocesarse y cuales afectan solo a la segmentacion
	// los nodos de re-preprocesarse es un subset de los nodos que deben segmentarse
	void getWorkToDo(queue<DefNode*>& preprocess, queue<DefNode*>& segmentation);

	// Preproceso completo, de otras veces.
	bool preprocessModelForComputations();

	// Guardar todas las posiciones de reposo para calculos de deformacion
	bool saveRestPoses();

	// Vuelve a poner las poses que teniamos antes de la animacion para seguir riggeando.
	bool restorePoses();

	// This functions changes de values and propagates the 
	// dirty flag properly.
	bool translateDefGroup(Vector3d newPos, int nodeId, bool update = false);
	bool rotateDefGroup(Quaterniond _q, int nodeId, bool update = false);
	bool rotateDefGroup(double rx, double ry, double rz, bool radians, int nodeId, bool update = false);
	bool changeExpansionValue(float value, int nodeId, bool update = false);
	bool changeSmoothValue(float value, int nodeId, bool update = false);

	void highlight(int _nodeId, bool hl);

	virtual bool propagateDirtyness()
    {
        dirtyFlag = true;

		for(int i = 0; i< defRig.roots.size(); i++)
			defRig.roots[i]->propagateDirtyness();

        return true;
    }

    virtual bool update()
    {
        if(!dirtyFlag)
            return true;
        else
		{
			for(int i = 0; i< defRig.roots.size(); i++)
			{
				defRig.roots[i]->transformation->computeWorldPos();
				defRig.roots[i]->update();
			}

			dirtyFlag = false;
            return true;
		}
    }

	//bool changeTwistValues(float ini, float fin, bool enable);

};

void BuildGroupTree(DefGraph& graph);

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
	virtual bool loadFromFile(ifstream& in)
	{ 
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

	virtual bool loadFromFile(ifstream& in)	
	{ 
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
	virtual bool loadFromFile(ifstream& in)	
	{ 
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
	virtual bool loadFromFile(ifstream& in)	
	{ 
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
	virtual bool loadFromFile(ifstream& in)	
	{ 
		return true;
	}
};

//FUNCTIONS FOR PROCESSING DATA STRUCTURES

// Propagate a expansión value, depending of the child associated
bool propagateExpansion(DefGroup& gr, float parentValue, int childId, float childValue);

// Propagate a expansión value, depending of the child associated
bool propagateExpansion(DefGroup& gr);

// Process just one skeleton adding the info to last loaded skeletons
bool processSkeleton(skeleton* skt, DefGraph& defRig, float subdivisions);

// Propose deformation nodes depending on the joint linked
bool proposeDefNodesFromStick(DefGroup& group, vector<DefGroup*> relatedGroups );

// Propose a division in deformers from a given stick
int subdivideStick(Vector3d origen, Vector3d fin, int defGroupIdx, int childDefGroupIdx,
				   vector< DefNode >& nodePoints, float subdivisionRatio);

// Propose a division in deformers from a given stick
int subdivideStick_FPMethod(Vector3d origen, Vector3d fin, int defGroupIdx, int childDefGroupIdx,
							vector< DefNode >& nodePoints, float subdivisionRatio);

// Takes all the groups and constraints and
// build a relation tree for weights computation
void BuildGroupTree(DefGraph& graph);

bool bindRigToScene(Modelo* model, vector<skeleton*>skeletons, AirRig* rig);

bool getBDEmbedding(Modelo* model);

#endif // AIR_RIG_H