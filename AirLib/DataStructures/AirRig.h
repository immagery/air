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

#include <DataStructures\DefGroup.h>
#include <DataStructures\DefGraph.h>
#include <DataStructures\AirConstraint.h>

#include <Eigen/Dense>


using namespace std;
using namespace Eigen;

enum riggingMode {MODE_RIG = 0, MODE_CREATE, MODE_ANIM, MODE_TEST};

// This clases define the dependency relationship between deformers,
class ControlGroup : public node
{
public:
	ControlGroup() {}
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

	virtual bool propagateDirtyness();

    virtual bool update();
	virtual void getNodeData(int nodeId, string& sName);

	//bool changeTwistValues(float ini, float fin, bool enable);
	virtual void getDataFromDefGroup(int element, float& expansion, bool& twistEnabled,
									 bool& bulgeEnabled,int& localSmoothPases,
									 float& twistIni,float& twistFin);

	virtual bool isSelectable(int element);

	void cleanDefNodesDirtyBit();


};

void BuildGroupTree(DefGraph& graph);

//FUNCTIONS FOR PROCESSING DATA STRUCTURES

// Propagate a expansión value, depending of the child associated
bool propagateExpansion(DefGroup& gr, float parentValue, int childId, float childValue);

// Propagate a expansión value, depending of the child associated
bool propagateExpansion(DefGroup& gr);

// Process just one skeleton adding the info to last loaded skeletons
bool processSkeleton(skeleton* skt, DefGraph& defRig, float subdivisions);

// Update nodes created inside the stick
bool updateDefNodesFromStick(DefGroup* group);

// Propose deformation nodes depending on the joint linked
bool proposeDefNodesFromStick(DefGroup& group, vector<DefGroup*> relatedGroups );

// Propose a division in deformers from a given stick
int subdivideStick(Vector3d origen, Vector3d fin, int defGroupIdx, int childDefGroupIdx,
				   vector< DefNode >& nodePoints, float subdivisionRatio, bool noNewId = false);

// Propose a division in deformers from a given stick
int subdivideStick_FPMethod(Vector3d origen, Vector3d fin, int defGroupIdx, int childDefGroupIdx,
							vector< DefNode >& nodePoints, float subdivisionRatio);

// Takes all the groups and constraints and
// build a relation tree for weights computation
void BuildGroupTree(DefGraph& graph);

bool bindRigToScene(Modelo* model, vector<skeleton*>skeletons, AirRig* rig);

bool getBDEmbedding(Modelo* model);

#endif // AIR_RIG_H