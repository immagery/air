#include "AirRig.h"
#include <DataStructures/scene.h>
#include <Computation\Segmentation.h>
#include <Computation\mvc_interiorDistances.h>
#include <DataStructures\InteriorDistancesData.h>


// Serialization
bool Constraint::saveToFile(FILE* fout)
{ 
	node::saveToFile(fout);

	fprintf(fout, "%d %d %f\n", parent->nodeId, child->nodeId, weight);
	fflush(fout);
	return true;
}


// How can a I load the references.
bool Constraint::loadFromFile(FILE* fout)
{
	node::loadFromFile(fout);

	// We save the id for restore later the references.
	float in_weight = 0;

	sConstraint = new constraintSerialization();
	fscanf(fout,"%d %d %f", &sConstraint->parentId, &sConstraint->childId, &in_weight);

	weight = in_weight;

	// need to update
	dirtyFlag = true;

	return true;
}

//////////////////
//   AIRRIG     //
//////////////////
bool AirRig::saveToFile(FILE* fout)
{
	if(!fout)
	{
		printf("There is no file to print!\n [AIR_RIG]");
		return false;
	}

	fprintf(fout, "%s\n", model->sName); fflush(fout);
	fprintf(fout, "%d\n", skeletons.size()); fflush(fout);
	for(int i = 0; i< skeletons.size(); i++)
		fprintf(fout, "%s\n", skeletons[i]->sName); fflush(fout);

	// Parameters for computation
	//double iniTwist;
	//double finTwist; 
	//bool enableTwist;
	fprintf(fout, "%f %f %d\n", iniTwist, finTwist, enableTwist); fflush(fout);

	defRig.saveToFile(fout);
	controlRig.saveToFile(fout);

	// Skinning 
	// I don't know if there are something to do here.
	fprintf(fout, "%d", skinning->useSecondaryWeights); fflush(fout);

	// All the relation with the model and the skeletons
	// should be done in the previous steps and bind it.

	return true;
}

// Loads data from this file, it is important to bind
// this data with the model and skeleton after this function.
bool AirRig::loadFromFile(FILE* fin)
{
	serializedData = new airRigSerialization();
	fscanf(fin, "%s", &serializedData->sModelName); 
	int skeletonsSize = 0;
	fscanf(fin, "%d", &skeletonsSize); 
	serializedData->sSkeletonName.resize(skeletonsSize);

	for(int i = 0; i< skeletonsSize; i++)
		fscanf(fin, "%s", serializedData->sSkeletonName);

	//double iniTwist;
	//double finTwist; 
	//bool enableTwist;
	fscanf(fin, "%f %f %d\n", &iniTwist, &finTwist, &enableTwist);

	// Rig System
	if(!defRig.loadFromFile(fin)) return false;

	// Control System
	if(!controlRig.loadFromFile(fin)) return false;

	// Skinning 
	// useSecondayWeights
	fscanf(fin, "%d", &skinning->useSecondaryWeights);

	return true;
}


bool getBDEmbedding(Modelo* model)
{	
	bool success = false;
	
	// Si no se ha calculado las distancias biharmonicas lo hacemos
    if(!model->computedBindings)
    {
        char bindingFileName[150];
        char bindingFileNameComplete[150];
        sprintf(bindingFileName, "%s/bind_%s", model->sPath.c_str(), model->sName.c_str());
        sprintf(bindingFileNameComplete, "%s.bin", bindingFileName);

        bool ascii = false;

        // A. Intentamos cargarlo
        ifstream myfile;
        myfile.open (bindingFileNameComplete, ios::in |ios::binary);
        bool loaded = false;
        if (myfile.is_open())
        {
            // En el caso de existir simplemente tenemos que cargar las distancias.
            loaded = LoadEmbeddings(*model, bindingFileNameComplete);
        }

        //B. En el caso de que no se haya cargado o haya incongruencias con lo real, lo recomputamos.
        if(!loaded)
        {
            success = ComputeEmbeddingWithBD(*model);
            if(!success)
            {
                printf("[ERROR - AirRig] No se ha conseguido computar el embedding\n");
                fflush(0);
                return false;
            }
            else 
				SaveEmbeddings(*model, bindingFileName, ascii);
        }
    }

	return success;
}

void AirRig::initParameters()
{
	iniTwist = 0.0; // To replace in the group parameters
	finTwist = 1.0; // To replace in the group parameters
	enableTwist = true; // To replace in the group parameters
}

// CONSTRUCTORES
// Default empty rig
AirRig::AirRig(int id) : rig(id)
{
	initParameters();

	skinning = new AirSkinning();
}

// An empty rig is created from a model
AirRig::AirRig(Modelo* in_model, int id) : rig(in_model, id)
{
	// We get the model
	//model = in_model;
	
	skinning = new AirSkinning();

	skinning->bindings.resize(1);
	skinning->bindings[0].push_back(model->bindings[0]);

	// Preprocesses like distances over the surface
	// could be done, maybe as a backgroud process.
	// The data path and files from the model need to be coherent
	getBDEmbedding(model);

	// Init all the parameters in the same place.
	initParameters();

}

// An default configured rig is created from a model and a set of skeletons
AirRig::AirRig(Modelo* in_model, vector<skeleton*>& skts, int id) : rig(in_model, skts, id)
{
	// We get the model
	model = in_model;

	skinning = new AirSkinning();
	skinning->bindings.resize(1);
	skinning->bindings[0].push_back(model->bindings[0]);

	// Preprocesses like distances over the surface
	// could be done, maybe as a backgroud process.
	// The data path and files from the model need to be coherent
	getBDEmbedding(model);

	// We save the skeleton for later processing
	skeletons.resize(skts.size()); 
	for(int sktIdx = 0; sktIdx< skts.size(); sktIdx++)
	{
		skeletons[sktIdx] = skts[sktIdx];
		processSkeleton(skeletons[sktIdx], defRig);
	}

	// Init all the parameters in the same place.
	initParameters();
}

AirRig::~AirRig()
{
}

//////////////////
//   DEFGRAPH   //
//////////////////

bool DefGraph::saveToFile(FILE* fout)
{
	if(!fout)
	{
		printf("There is no file to print!\n [AIR_RIG]");
		return false;
	}

	node::saveToFile(fout);

	fprintf(fout, "%d %d\n", defGroups.size(), relations.size()); fflush(fout);
	for(int groupIdx = 0; groupIdx < defGroups.size(); groupIdx++)
	{
		defGroups[groupIdx]->saveToFile(fout);
	}

	for(int relationIdx = 0; relationIdx < relations.size(); relationIdx++)
	{
		relations[relationIdx]->saveToFile(fout);
	}

	/* 
		Important to ensure the data consistency
		vector<DefGroup*> roots; -> building the tree
		vector<joint*> joints; -> binding the skeletons
		vector<DefNode*> deformers; -> looking to the groups
		map<unsigned int, DefGroup*> defGroupsRef;
		map<unsigned int, DefNode*> defNodesRef;
	*/

	return true;
}

// Loads data from this file, it is important to bind
// this data with the model and skeleton after this function.
bool DefGraph::loadFromFile(FILE* fout)
{
	if(!fout)
	{
		printf("There is no file to print!\n [AIR_RIG]");
		return false;
	}

	node::loadFromFile(fout);

	int relationsSize;
	int defGroupsSize;

	fscanf(fout, "%d %d\n", defGroupsSize, relationsSize); 

	defGroups.resize(defGroupsSize);
	for(int groupIdx = 0; groupIdx < defGroups.size(); groupIdx++)
	{
		defGroups[groupIdx] = new DefGroup(scene::getNewId());
		if(!defGroups[groupIdx]->loadFromFile(fout)) return false;
	}

	relations.resize(relationsSize);
	for(int relationIdx = 0; relationIdx < relations.size(); relationIdx++)
	{
		relations[relationIdx] = new Constraint();
		if(!relations[relationIdx]->loadFromFile(fout)) return false;
	}

	/* 
		Important to ensure the data consistency
		vector<DefGroup*> roots; -> building the tree
		vector<joint*> joints; -> binding the skeletons
		vector<DefNode*> deformers; -> looking to the groups
		map<unsigned int, DefGroup*> defGroupsRef;
		map<unsigned int, DefNode*> defNodesRef;
	*/

	return true;
}

//////////////////
//   DEFGROUP   //
//////////////////
DefGroup::DefGroup(int nodeId) : node(nodeId)
{
	transformation = NULL;
	smoothingPasses = default_SMOOTHING_PASES;
	smoothPropagationRatio = default_SMOOTHING_RATIO;
	subdivisionRatio = default_SUBDIVISION_RATIO;
	expansion = default_EXPANSION;
}

DefGroup::DefGroup(int nodeId, joint* jt) : node(nodeId)
{
	transformation = jt;

	smoothingPasses = default_SMOOTHING_PASES;
	smoothPropagationRatio = default_SMOOTHING_RATIO;
	subdivisionRatio = default_SUBDIVISION_RATIO;
	expansion = default_EXPANSION;
}

bool DefGroup::saveToFile(FILE* fout)
{
	if(!fout)
	{
		printf("There is no file to print!\n [AIR_RIG]");
		return false;
	}

	node::saveToFile(fout);

	fprintf(fout, "%d\n", deformers.size()); fflush(fout);
	fprintf(fout, "%s\n", transformation->sName); fflush(fout);
	fprintf(fout, "%f %f %d %f\n", subdivisionRatio, expansion, smoothingPasses,smoothPropagationRatio); fflush(fout);
	fprintf(fout, "%d\n", type); fflush(fout);

	for(int i = 0; i< deformers.size(); i++)
		deformers[i].saveToFile(fout);

	// For computational pourposes.
	//vector<DefGroup*> relatedGroups;

	return true;
}

// Loads data from this file, it is important to bind
// this data with the model and skeleton after this function.
bool DefGroup::loadFromFile(FILE* fout)
{
	int deformersSize;
	fscanf(fout, "%d", &deformersSize);

	serializedData = new defGroupSerialization();

	fscanf(fout, "%s", &serializedData->sJointName);
	fscanf(fout, "%f %f %d %f", &subdivisionRatio, &expansion, &smoothingPasses,&smoothPropagationRatio);
	fscanf(fout, "%d", &type); 

	for(int i = 0; i< deformers.size(); i++)
		if(!deformers[i].loadFromFile(fout)) return false;

	return true;
}



/// FUNCTIONS FOR PROCESSING DATA

bool processSkeleton(skeleton* skt, DefGraph& defRig)
{
	map <unsigned int,unsigned int> jointIndexes;	
	
	//defRig.defGroups.resize(skt->joints.size());
	for(int jointIdx = 0; jointIdx < skt->joints.size(); jointIdx++)
	{
		int currentDefgroup = defRig.defGroups.size();
		defRig.defGroups.push_back(new DefGroup(scene::getNewId(), skt->joints[jointIdx]));
		
		// definimos algunos valores de comportamiento, el joint no tiene porque tener esta info.
		defRig.defGroups.back()->expansion = 1;

		jointIndexes[skt->joints[jointIdx]->nodeId] = currentDefgroup;
	}

	for(int jointIdx = 0; jointIdx < skt->joints.size(); jointIdx++)
	{
		vector<DefGroup*> relatedGroups;
		for(int childIdx = 0; childIdx < skt->joints[jointIdx]->childs.size(); childIdx++)
		{
			int parentId = jointIndexes[skt->joints[jointIdx]->nodeId];
			int childId = jointIndexes[skt->joints[jointIdx]->childs[childIdx]->nodeId];

			defRig.relations.push_back(new Constraint());
			defRig.relations.back()->parent = defRig.defGroups[parentId];
			defRig.relations.back()->child = defRig.defGroups[childId];
			defRig.relations.back()->weight = 1.0;
			relatedGroups.push_back(defRig.relations.back()->child);
		}
		proposeDefNodesFromStick(*defRig.defGroups[jointIdx], relatedGroups);

		for(int defId = 0; defId < defRig.defGroups[jointIdx]->deformers.size(); defId++)
			defRig.deformers.push_back(&defRig.defGroups[jointIdx]->deformers[defId]);
	}

	return true;
}

bool proposeDefNodesFromStick(DefGroup& group, vector<DefGroup*> relatedGroups )
{
	joint* jt = group.transformation;
	if(jt->dirtyFlag)
		jt->update();

	// Limpiamos los deformadores que pudieran existir antes
	group.deformers.clear();

	group.deformers.push_back(DefNode(jt->getWorldPosition(), jt->nodeId));
	group.deformers.back().nodeId = scene::getNewId();
	group.deformers.back().ratio = 0.0;
	group.deformers.back().expansion = group.expansion;
	group.deformers.back().enableWeightsComputation = jt->enableWeightsComputation;

	// We link all the child nodes to this node.
	for(int i = 0; i < jt->getChildCount(); i++)
	{
		int numDivisions;
		numDivisions = subdivideStick(jt->getWorldPosition(), jt->childs[i]->getWorldPosition(), 
									  group.nodeId, relatedGroups[i]->nodeId,
									  group.deformers, group.subdivisionRatio);

		propagateExpansion(group, group.expansion, relatedGroups[i]->nodeId, relatedGroups[i]->expansion);
	}

	return true;
}

// Propagate a expansión value, depending of the child associated
bool propagateExpansion(DefGroup& gr, float parentValue, int childId, float childValue)
{
	for(int group = 0; group < gr.deformers.size(); group++)
	{	
		if(gr.deformers[group].childBoneId != childId)
			continue;

		// Expansion propagation
		float expValue = parentValue;
		float expValue2 = childValue;

		//float expValue = parent->expansion;
		//float expValue2 = child->expansion;

		// Queda por decidir si queremos continuidad entre las expansiones o no.
		float ratio2 = gr.deformers[group].ratio/ratioExpansion_DEF;
		if(ratio2 > 1) ratio2 = 1;
		if(ratio2 < 0) ratio2 = 0;

        float dif = 1-expValue;
        float newValue =  expValue + dif*ratio2;

		gr.deformers[group].expansion = newValue;
	}

	return true;
}

int subdivideStick(Vector3d origen, Vector3d fin, int defGorupIdx, int childDefGorupIdx,
				   vector< DefNode >& nodePoints, float subdivisionRatio)
{
	//float subdivisionRatio = subdivisionRatio_DEF;
	//Vector3d origen =  parent->getWorldPosition();
	//Vector3d fin = child->getWorldPosition();
	int boneId = defGorupIdx;

	double longitud= (float)((fin-origen).norm());
	double endChop = longitud*endChop_DEF;

	if(longitud == 0)
		return 0;

	Vector3d dir = (fin-origen)/longitud;
	longitud = longitud - endChop;
	int numDivisions = (int)floor(longitud/subdivisionRatio);

	double newSubdivLength = longitud;
	if(numDivisions > 0)
		newSubdivLength = longitud/ double(numDivisions);

	Vector3d newOrigen = origen;
	Vector3d newFin = fin-dir*endChop;

	// Añadimos los nodos
	// Saltamos el primer nodo que corresponde a la raíz del joint
	for(int i = 1; i<= numDivisions; i++)
	{
		//int globalNodeId = nodePoints.size();
		nodePoints.push_back(DefNode(newOrigen+(dir*newSubdivLength*i),boneId));
		nodePoints.back().nodeId = scene::getNewId();
		nodePoints.back().ratio = (float)i/(float)numDivisions;

		nodePoints.back().childBoneId = childDefGorupIdx;
	}

	if(VERBOSE)
	{
		float error = (float)(newFin - nodePoints.back().pos).norm();
		if(longitud>subdivisionRatio && fabs(error) > pow(10.0, -5))
		{
			// TODEBUG
			printf("numDivisions: %d, subdivisionRatio: %f, newSubdivLength: %f\n", numDivisions, subdivisionRatio, newSubdivLength);
			printf("Tenemos un posible error de calculo de nodos en huesos: %f\n", error);
			printf("y la longitud es: %f\n", longitud);
			fflush(0);
		}
	}

	return numDivisions+1;
}

bool AirRig::loadRigging(string sFile)
{
	FILE* fin = NULL;
	fin = fopen(sFile.c_str(),"r");
	if(fin) loadFromFile(fin);

	fclose(fin);
	return true;
}

void BuildGroupTree(DefGraph& graph)
{
	graph.defNodesRef.clear();

	map<unsigned int, DefGroup*>& groups = graph.defGroupsRef;
	for(int grIdx = 0; grIdx < graph.defGroups.size(); grIdx++)
	{
		groups[graph.defGroups[grIdx]->nodeId] = graph.defGroups[grIdx];
		graph.defGroups[grIdx]->relatedGroups.clear();

		for(int nodeIdx = 0; nodeIdx < graph.defGroups[grIdx]->deformers.size(); nodeIdx++)
		{
			graph.defNodesRef[graph.defGroups[grIdx]->deformers[nodeIdx].nodeId] = &graph.defGroups[grIdx]->deformers[nodeIdx];
		}
	}

	map<int, bool> parented;
	for(int ctrIdx = 0; ctrIdx < graph.relations.size(); ctrIdx++)
	{
		// there are serveral types of constraint. Depending on what constraint is defined,
		// the computation tree is updated.
		graph.relations[ctrIdx]->parent->relatedGroups.push_back(graph.relations[ctrIdx]->child);
		parented[graph.defGroupsRef[graph.relations[ctrIdx]->child->nodeId]->nodeId] = true;
	}

	for(int grIdx = 0; grIdx < graph.defGroups.size(); grIdx++)
	{
		// If there is no parent from this joint it will be classified as root
		if(parented.count(graph.defGroups[grIdx]->nodeId) == 0)
		{
			graph.roots.push_back(graph.defGroups[grIdx]);
		}
	}
}

bool AirRig::bindRigToScene(Modelo& model, vector<skeleton*>& skeletons)
{
	AirRig* rig;

	// bind model
	rig->bindModel(model);
	
	//bind skeletons
	for(int i = 0; i< skeletons.size(); i++)
		rig->skeletons.push_back(skeletons[i]);

	// get joint info
	map<string, joint*> allJoints;
	for(int i = 0; i< skeletons.size(); i++)
	{
		for(int idJoint = 0; idJoint < skeletons[i]->joints.size(); idJoint++)
		{
			allJoints[skeletons[i]->joints[idJoint]->sName] = skeletons[i]->joints[idJoint];
		}
	}
	
	// bind Groups to joints -> to remove???
	map<int, int > nodeCorrespondence;
	for(int defgroupIdx = 0; defgroupIdx< rig->defRig.defGroups.size(); defgroupIdx++)
	{
		// get correspondence
		nodeCorrespondence[rig->defRig.defGroups[defgroupIdx]->sNode->nodeId] = rig->defRig.defGroups[defgroupIdx]->nodeId;
		rig->defRig.defGroupsRef[rig->defRig.defGroups[defgroupIdx]->nodeId] = rig->defRig.defGroups[defgroupIdx];

		// bind to the transformation
		joint* jt = NULL;
		rig->defRig.defGroups[defgroupIdx]->transformation = allJoints[rig->defRig.defGroups[defgroupIdx]->serializedData->sJointName];
		if(!rig->defRig.defGroups[defgroupIdx]->transformation) return false;
		delete rig->defRig.defGroups[defgroupIdx]->serializedData;
		rig->defRig.defGroups[defgroupIdx]->serializedData = NULL;

		// get correspondence
		for(int defIdx = 0; defIdx< rig->defRig.defGroups[defIdx]->deformers.size(); defIdx++)
		{
			nodeCorrespondence[rig->defRig.defGroups[defgroupIdx]->deformers[defIdx].sNode->nodeId] = rig->defRig.defGroups[defgroupIdx]->deformers[defIdx].nodeId;
		}
	}

	// Bind Constraint Relation
	for(int relationIdx = 0; relationIdx< rig->defRig.relations.size(); relationIdx++)
	{
		int childIdx = nodeCorrespondence[rig->defRig.relations[relationIdx]->sConstraint->childId];
		int parentIdx = nodeCorrespondence[rig->defRig.relations[relationIdx]->sConstraint->parentId];

		rig->defRig.relations[relationIdx]->child = rig->defRig.defGroupsRef[childIdx];
		rig->defRig.relations[relationIdx]->parent = rig->defRig.defGroupsRef[parentIdx] ;
	}

	BuildGroupTree(rig->defRig);

	//rig->skinning.bindings[0] = vector<binding*> ();
	//for(int i = 0; i< rig->model->bindings.size(); i++)
	//	rig->skinning.bindings[0].push_back(rig->model->bindings[i]);

	rig->skinning->deformedModels.push_back(rig->model);
	rig->skinning->originalModels.push_back(rig->model->originalModel);
	rig->skinning->rig = rig;

	// Default initialization
	rig->skinning->bind = new binding();

	return false;
}
