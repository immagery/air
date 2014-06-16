#include "AirRig.h"
#include <DataStructures/scene.h>
#include <Computation\AirSegmentation.h>
#include <Computation\mvc_interiorDistances.h>
#include <DataStructures\InteriorDistancesData.h>

#include <iostream>
#include <fstream>
#include <queue>

#define FP_METHOD false

riggingMode AirRig::mode = MODE_RIG;


//////////////////
//   AIRRIG     //
//////////////////

// CONSTRUCTORES
// Default empty rig
AirRig::AirRig(int id) : Rig(id)
{
	initParameters();
	if(skin) delete skin;
	airSkin = new AirSkinning();
	skin = (Skinning*) airSkin;
}

AirRig::~AirRig()
{
	delete skin;
}


bool AirRig::saveToFile(FILE* fout)
{
	if(!fout)
	{
		printf("There is no file to print!\n [AIR_RIG]");
		return false;
	}

	fprintf(fout, "%s\n", model->sName.c_str()); fflush(fout);
	fprintf(fout, "%d\n", skeletons.size()); fflush(fout);
	for(int i = 0; i< skeletons.size(); i++)
		fprintf(fout, "%s\n", skeletons[i]->sName.c_str()); fflush(fout);

	// Parameters for computation
	//double iniTwist;
	//double finTwist; 
	//bool enableTwist;
	fprintf(fout, "%f %f %d\n", iniTwist, finTwist, enableTwist); fflush(fout);

	defRig.saveToFile(fout);
	controlRig.saveToFile(fout);

	// Skinning 
	// I don't know if there are something to do here.
	fprintf(fout, "%d", skin->useSecondaryWeights); fflush(fout);

	// All the relation with the model and the skeletons
	// should be done in the previous steps and bind it.

	return true;
}


// Loads data from this file, it is important to bind
// this data with the model and skeleton after this function.
bool AirRig::loadFromFile(ifstream& in)
{
	serializedData = new airRigSerialization();

	string line;
	vector<string> elems;

	getline (in , line);
	serializedData->sModelName = line;

	int skeletonsSize = 0;
	getline (in , line);
	split(line, ' ', elems);
	skeletonsSize = atoi(elems[0].c_str());
	
	serializedData->sSkeletonName.resize(skeletonsSize);

	for(int i = 0; i< skeletonsSize; i++)
	{
		getline (in , line);
		serializedData->sSkeletonName[i] = line;
	}

	//double iniTwist;
	//double finTwist; 
	//bool enableTwist;
	getline (in , line);
	split(line, ' ', elems);
	iniTwist = atof(elems[0].c_str());
	finTwist = atof(elems[1].c_str());
	enableTwist = atoi(elems[2].c_str());

	//map<int, int> defNodesRelation;
	//map<int, int> defGroupsRelation;

	// Rig System
	if(!defRig.loadFromFile(in, serializedData)) return false;

	// Control System
	if(!controlRig.loadFromFile(in)) return false;

	// Skinning 
	// useSecondayWeights
	getline (in , line);
	split(line, ' ', elems);
	skin->useSecondaryWeights = atoi(elems[0].c_str());

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
			model->computedBindings = true;
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

			model->computedBindings = true;
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


/*
// An empty rig is created from a model
AirRig::AirRig(Modelo* in_model, int id) : rig(in_model, id)
{
	// We get the model

	skinning = new AirSkinning();
	skinning->bind = model->bind;

	// Preprocesses like distances over the surface
	// could be done, maybe as a backgroud process.
	// The data path and files from the model need to be coherent
	getBDEmbedding(model);

	// Init all the parameters in the same place.
	initParameters();

}
*/

/*

// An default configured rig is created from a model and a set of skeletons
AirRig::AirRig(Modelo* in_model, vector<skeleton*>& skts, int id) : rig(in_model, skts, id)
{
	// We get the model
	model = in_model;

	skinning = new AirSkinning();

	// Init all the parameters in the same place.
	initParameters();
}
*/

bool AirRig::update()
{
    if(!dirtyFlag)
        return true;
    else
	{
		for(int i = 0; i< defRig.defGroups.size(); i++)
		{
			//defRig.roots[i]->transformation->computeWorldPos();
			defRig.defGroups[i]->update();
		}

		//Reconstruimos la lista de nodos.
		defRig.deformers.clear();
		defRig.defNodesRef.clear();

		// Evaluamos si hay alguna incongruencia con los deformadores para computar.
		for(int dgIdx = 0; dgIdx < defRig.defGroups.size(); dgIdx++)
		{
			for(int dnIdx = 0; dnIdx < defRig.defGroups[dgIdx]->deformers.size(); dnIdx++)
			{
				// Apilamos todos los nodos... luego el sistema se encarga de despreciar lo que molesta.
				DefNode* dn = &(defRig.defGroups[dgIdx]->deformers[dnIdx]);
				defRig.deformers.push_back(dn);
				defRig.defNodesRef[defRig.defGroups[dgIdx]->deformers[dnIdx].nodeId] = dn;
			}
		}

		dirtyFlag = false;
        return true;
	}
}

void AirRig::getNodeData(int nodeId, string& sName)
{
	for(int i = 0; i< defRig.defGroups.size(); i++)
	{
		if(defRig.defGroups[i]->nodeId == nodeId)
		{
			DefGroup* dg =  defRig.defGroups[i];
			sName = dg->sName;
		}
	}
}

void AirRig::getDataFromDefGroup(int element, float& expansion, bool& twistEnabled,
									 bool& bulgeEnabled,int& localSmoothPases,
									 float& twistIni,float& twistFin)
{
	for(int i = 0; i< defRig.defGroups.size(); i++)
	{
		if(defRig.defGroups[i]->nodeId == element)
		{
			DefGroup* dg =  defRig.defGroups[i];
			expansion = dg->expansion;
			twistEnabled = dg->enableTwist;
			twistIni = dg->iniTwist;
			twistFin = dg->finTwist;
			localSmoothPases = dg->localSmooth;
			bulgeEnabled = dg->bulgeEffect;
		}
	}	
}

bool AirRig::isSelectable(int element)
{
	for(int i = 0; i< defRig.defGroups.size(); i++)
	{
		if(defRig.defGroups[i]->nodeId == element)
			return true;
	}

	return false;
}

bool AirRig::propagateDirtyness()
{
    dirtyFlag = true;

	for(int i = 0; i< defRig.roots.size(); i++)
		defRig.roots[i]->propagateDirtyness();

    return true;
}

/// FUNCTIONS FOR PROCESSING DATA

bool processSkeleton(skeleton* skt, DefGraph& defRig, float subdivisions)
{
	map <unsigned int,unsigned int> jointIndexes;	
	
	//defRig.defGroups.resize(skt->joints.size());
	for(int jointIdx = 0; jointIdx < skt->joints.size(); jointIdx++)
	{
		int currentDefgroup = defRig.defGroups.size();
		defRig.defGroups.push_back(new DefGroup(scene::getNewId(T_DEFGROUP), skt->joints[jointIdx]));
		defRig.defGroups.back()->sName = string("DGrp_") + skt->joints[jointIdx]->sName;
		defRig.defGroups.back()->subdivisionRatio = subdivisions;
		defRig.defGroups.back()->references = &(defRig.defGroupsRef);
		
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

bool updateDefNodesFromStick(DefGroup* group )
{
	vector<DefGroup*>& relatedGroups = group->relatedGroups;

	joint* jt = group->transformation;
	if(jt->dirtyFlag) jt->update();

	// Limpiamos los deformadores que pudieran existir antes
	int deformersPoolSize = group->deformers.size();

	// The first don't need to be relocated

	// We get the deleted deformer memory position to ensure memory coherence.
	vector<int> cleanDeformers;

	// Si no hay elementos ponemos el primero.
	if(group->deformers.size() == 0)
	{
		group->deformers.push_back(DefNode(jt->translation, group->nodeId));
		group->deformers.back().nodeId = scene::getNewId(T_DEFNODE);
	}

	DefNode& def = group->deformers[0];
	def.boneId = group->nodeId;
	def.pos = jt->translation;
	def.relPos = def.pos - jt->translation;
	def.ratio = 0.0;
	def.expansion = group->expansion;
	def.enableWeightsComputation = jt->enableWeightsComputation;
	def.freeNode = false;
	def.segmentationDirtyFlag = true;

	// We set as clean the other nodes.
	for(int i = 1; i< deformersPoolSize; i++)
	{
		group->deformers[i].freeNode = true;
		group->deformers[i].segmentationDirtyFlag = true;
		cleanDeformers.push_back(i);
	}

	// We propose new defNodes for each branch
	vector<DefNode> deformersProposal;
	for(int i = 0; i < group->relatedGroups.size(); i++)
	{
		subdivideStick( group->transformation->translation, 
						group->relatedGroups[i]->transformation->translation, 
						group->nodeId, relatedGroups[i]->nodeId,
						deformersProposal, group->subdivisionRatio, true);
	}

	// We copy the data form proposed nodes to free node spaces.
	if(deformersProposal.size() > cleanDeformers.size())
	{
		int numOfAddings = deformersProposal.size()- cleanDeformers.size();
		for(int i = 0; i < numOfAddings; i++)
		{
			cleanDeformers.push_back(group->deformers.size());
			group->deformers.push_back(DefNode(Vector3d(0,0,0),group->nodeId));
			group->deformers.back().nodeId = scene::getNewId(T_DEFNODE);
			group->deformers.back().freeNode = true;
			group->deformers.back().addedToComputations = false;
			group->deformers.back().segmentationDirtyFlag = true;
		}
	}

	assert(deformersProposal.size() <= cleanDeformers.size());

	for(int i = 0; i < deformersProposal.size(); i++)
	{
		group->deformers[cleanDeformers[i]].copyKeyInfoFrom(deformersProposal[i]);
		group->deformers[cleanDeformers[i]].freeNode = false;
		group->deformers[cleanDeformers[i]].segmentationDirtyFlag = true;
	}

	for(int i = 0; i < jt->getChildCount(); i++)
	{
		propagateExpansion(*group, group->expansion, relatedGroups[i]->nodeId, relatedGroups[i]->expansion);
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

	group.deformers.push_back(DefNode(jt->translation, group.nodeId));
	group.deformers.back().relPos = group.deformers.back().pos-jt->translation;
	group.deformers.back().nodeId = scene::getNewId(T_DEFNODE);
	group.deformers.back().ratio = 0.0;
	group.deformers.back().expansion = group.expansion;
	group.deformers.back().enableWeightsComputation = jt->enableWeightsComputation;
	group.deformers.back().freeNode = false;

	// We link all the child nodes to this node.
	for(int i = 0; i < jt->getChildCount(); i++)
	{
		int numDivisions;

		
		if(FP_METHOD)
		{
			numDivisions = subdivideStick_FPMethod(jt->getWorldPosition(), jt->childs[i]->getWorldPosition(), 
										  group.nodeId, relatedGroups[i]->nodeId,
										  group.deformers, group.subdivisionRatio);
		}
		else
		{
		
			numDivisions = subdivideStick(jt->getWorldPosition(), jt->childs[i]->getWorldPosition(), 
										  group.nodeId, relatedGroups[i]->nodeId,
										  group.deformers, group.subdivisionRatio);
		}

		propagateExpansion(group, group.expansion, relatedGroups[i]->nodeId, relatedGroups[i]->expansion);
	}

	return true;
}

DefGroup* getChildFromGroupId(DefGroup* gr, int id)
{
	for(int i = 0; i< gr->relatedGroups.size(); i++)
	{
		if(gr->relatedGroups[i]->nodeId == id)
			return gr->relatedGroups[i];
	}

	return NULL;
}

// Propagate a expansión value, depending of the child associated
bool propagateExpansion(DefGroup& gr)
{
	for(int group = 0; group < gr.deformers.size(); group++)
	{	
		DefGroup* child = getChildFromGroupId(&gr, gr.deformers[group].childBoneId);

		if(child == NULL)
		{
			gr.deformers[group].expansion = gr.expansion;
			gr.deformers[group].segmentationDirtyFlag = true;
			continue;
		}

		// Expansion propagation
		float expValue = gr.expansion;
		float expValue2 = child->expansion;

		//float expValue = parent->expansion;
		//float expValue2 = child->expansion;

		// Queda por decidir si queremos continuidad entre las expansiones o no.
		float ratio2 = gr.deformers[group].ratio/ratioExpansion_DEF;
		if(ratio2 > 1) ratio2 = 1;
		if(ratio2 < 0) ratio2 = 0;

        float dif = 1-expValue;
        float newValue =  expValue + dif*ratio2;

		//gr.deformers[group].expansion = newValue;
		gr.deformers[group].expansion = gr.expansion;
		gr.deformers[group].segmentationDirtyFlag = true;
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

		gr.deformers[group].relPos = gr.transformation->rotation.inverse()._transformVector(gr.deformers[group].pos- gr.transformation->translation);
	}

	return true;
}

int subdivideStick_FPMethod(Vector3d origen, Vector3d fin, int defGorupIdx, int childDefGorupIdx,
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

	nodePoints.push_back(DefNode(newOrigen+(dir*longitud*0.20),boneId));
	nodePoints.back().nodeId = scene::getNewId(T_DEFNODE);
	nodePoints.back().ratio = 0.20;
	nodePoints.back().childBoneId = childDefGorupIdx;
	nodePoints.back().freeNode = false;

	nodePoints.push_back(DefNode(newOrigen+(dir*longitud*0.50),boneId));
	nodePoints.back().nodeId = scene::getNewId(T_DEFNODE);
	nodePoints.back().ratio = 0.50;
	nodePoints.back().childBoneId = childDefGorupIdx;
	nodePoints.back().freeNode = false;

	nodePoints.push_back(DefNode(newOrigen+(dir*longitud*0.8),boneId));
	nodePoints.back().nodeId = scene::getNewId(T_DEFNODE);
	nodePoints.back().ratio = 0.8;
	nodePoints.back().childBoneId = childDefGorupIdx;
	nodePoints.back().freeNode = false;

	nodePoints.push_back(DefNode(newOrigen+(dir*longitud),boneId));
	nodePoints.back().nodeId = scene::getNewId(T_DEFNODE);
	nodePoints.back().ratio = 1.0;
	nodePoints.back().childBoneId = childDefGorupIdx;
	nodePoints.back().freeNode = false;

	return 5;
}


int subdivideStick(Vector3d origen, Vector3d fin, int defGorupIdx, int childDefGorupIdx,
				   vector< DefNode >& nodePoints, float subdivisionRatio, bool noNewId)
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
		if(!noNewId) nodePoints.back().nodeId = scene::getNewId(T_DEFNODE);
		nodePoints.back().ratio = (float)i/(float)numDivisions;
		nodePoints.back().freeNode = false;
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

	ifstream in(sFile.c_str());
	if(in.is_open())  loadFromFile(in);
	in.close();
	return true;
}

bool AirRig::translateDefGroup(Vector3d newPos, int nodeId, bool update)
{
	// Escoger transformación como punto simple o como jerarquía...
	// de entrada algo normal.
	if(!defRig.defGroupsRef[nodeId]) return false;
	
	// The defgraph is in this rig
	defRig.defGroupsRef[nodeId]->transformation->pos = newPos;

	 // parent and all the childs
	defRig.defGroupsRef[nodeId]->dirtyByTransformation(true);

	// Relanzar el cálculo completo: presuponemos que se ha actualizado bien el grafo
	// The deformers structure will be updated
	if(update)
		updateAirSkinning(defRig, *model);

	return true;
}


bool AirRig::rotateDefGroup(Quaterniond _q, int nodeId, bool update)
{
		// Escoger transformación como punto simple o como jerarquía...
	// de entrada algo normal.
	if(!defRig.defGroupsRef[nodeId]) return false;
	
	// The defgraph is in this rig
	defRig.defGroupsRef[nodeId]->transformation->setRotation( _q);

	 // parent and all the childs
	defRig.defGroupsRef[nodeId]->dirtyByTransformation(false);

	// Relanzar el cálculo completo: presuponemos que se ha actualizado bien el grafo
	// The deformers structure will be updated
	if(update)
		updateAirSkinning(defRig, *model);

	return true;
}

bool AirRig::rotateDefGroup(double rx, double ry, double rz, bool radians, int nodeId, bool update)
{
		// Escoger transformación como punto simple o como jerarquía...
	// de entrada algo normal.
	if(!defRig.defGroupsRef[nodeId]) return false;
	
	// The defgraph is in this rig
	defRig.defGroupsRef[nodeId]->transformation->setRotation( rx, ry, rz, radians);

	 // parent and all the childs
	defRig.defGroupsRef[nodeId]->dirtyByTransformation(false);

	// Relanzar el cálculo completo: presuponemos que se ha actualizado bien el grafo
	// The deformers structure will be updated
	if(update)
		updateAirSkinning(defRig, *model);

	return true;
}

bool AirRig::changeExpansionValue(float value, int nodeId, bool update)
{
	// set expansion value
	defRig.defGroupsRef[nodeId]->expansion = value;

	// Expand the value over the deformers

	 // parent and all the childs
	defRig.defGroupsRef[nodeId]->dirtyBySegmentation();

	// Relanzar el cálculo completo: presuponemos que se ha actualizado bien el dirtybit en el grafo
	// The deformers structure will be updated
	updateAirSkinning(defRig, *model);

	return true;
}

bool AirRig::changeSmoothValue(float value, int nodeId, bool update)
{
	defRig.defGroupsRef[nodeId]->smoothingPasses = value;
	defRig.defGroupsRef[nodeId]->localSmooth = true;

	updateAirSkinning(defRig, *model);
	return true;
}

void AirRig::highlight(int _nodeId, bool hl)
{
	for(int i = 0; i< defRig.defGroups.size(); i++)
	{
		if(defRig.defGroups[i]->nodeId == _nodeId)
			defRig.defGroups[i]->shading->highlight = hl;
		else
			defRig.defGroups[i]->shading->highlight = false;
	}
	

}

void BuildGroupTree(DefGraph& graph)
{
	graph.defNodesRef.clear();

	map<unsigned int, DefGroup*>& groups = graph.defGroupsRef;
	for(int grIdx = 0; grIdx < graph.defGroups.size(); grIdx++)
	{
		groups[graph.defGroups[grIdx]->nodeId] = graph.defGroups[grIdx];
		graph.defGroups[grIdx]->relatedGroups.clear();
		graph.defGroups[grIdx]->dependentGroups.clear();

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
		graph.defGroupsRef[graph.relations[ctrIdx]->child->nodeId]->dependentGroups.push_back(graph.defGroupsRef[graph.relations[ctrIdx]->parent->nodeId]);
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

bool AirRig::saveRestPoses()
{
	for(int j = 0; j < defRig.roots.size(); j++) 
	{
		defRig.roots[j]->saveRestPos(defRig.roots[j]);
		//defRig.roots[j]->computeRestPos(defRig.roots[j]);
	}
	return true;
}

bool AirRig::restorePoses()
{
	for(int j = 0; j < defRig.roots.size(); j++) 
	{
		defRig.roots[j]->restorePoses(defRig.roots[j]);
		defRig.roots[j]->computeWorldPos(defRig.roots[j]);
	}
	return true;
}

bool AirRig::preprocessModelForComputations()
{
	// embedding for computations
	// If it's not computed will be processed
	getBDEmbedding(model);

	// The deformers structure will be updated
	BuildGroupTree(defRig);

	return true;
}

void AirRig::initRigWithModel(Modelo* in_model)
{
	if(in_model->rigBinded) return;

	bindModel(in_model);

	// Preprocesses like distances over the surface
	// could be done, maybe as a backgroud process.
	// The data path and files from the model need to be coherent
	getBDEmbedding(model);

	// Se podría constuir todo el grafo de conectividad y estructuras para el proceso aquí.

	in_model->rigBinded = true;
}

void AirRig::RefineGroup(DefGroup* def)
{

}

void AirRig::BuildGroup(DefGroup* def)
{
		//defRig.defGroups[i]->subdivisionRatio = subdivisions;
		def->expansion = 1;

		// Constraint basico de relacion
		if(def->dependentGroups.size() > 0)
		{
			defRig.relations.push_back(new Constraint());
			defRig.relations.back()->parent = defRig.defGroupsRef[def->dependentGroups[0]->nodeId];
			defRig.relations.back()->child = defRig.defGroupsRef[def->nodeId];
			defRig.relations.back()->weight = 1.0;
		}

		// habría que ver si simplemente hay que reposicionar o reconstruir
		
		if(def->type == DEF_STICK)
		{
			proposeDefNodesFromStick(*def, def->relatedGroups);
		}
		else
		{
			printf("No conozco el tipo de este deformador.\n");
		}

		for(int defId = 0; defId < def->deformers.size(); defId++)
			defRig.deformers.push_back(&def->deformers[defId]);
}

void AirRig::getWorkToDo(queue<DefNode*>& preprocess, queue<DefNode*>& segmentation)
{
	// Revisar los defGroups para ver si hay que reposicionar los nodos o que
	for(int i = 0; i < defRig.defGroups.size(); i++)
	{
		if(defRig.defGroups[i]->dirtyTransformation)
		{
			for(int defId = 0; defId < defRig.defGroups[i]->deformers.size(); defId++)
			{
				preprocess.push(&defRig.defGroups[i]->deformers[defId]);
				segmentation.push(&defRig.defGroups[i]->deformers[defId]);
			}
		}
		else if(defRig.defGroups[i]->dirtySegmentation)
		{
			for(int defId = 0; defId < defRig.defGroups[i]->deformers.size(); defId++)
			{
				// Solo hay que hacer el cálculo de segmentacion porque se ha tocado algun parametro
				segmentation.push(&defRig.defGroups[i]->deformers[defId]);
			}
		}
		else
		{
			// Solo tenemos que hacer el smoothing
			// de momento no guardamos ninguna referencia
		}
	}
}


bool AirRig::bindRigToModelandSkeleton(Modelo* in_model, vector<skeleton*>& in_skeletons, float subdivisions)
{
	bindModel(in_model);

	// Preprocesses like distances over the surface
	// could be done, maybe as a backgroud process.
	// The data path and files from the model need to be coherent
	getBDEmbedding(model);

	// We save the skeleton for later processing
	skeletons.resize(in_skeletons.size()); 
	for(int sktIdx = 0; sktIdx< skeletons.size(); sktIdx++)
	{
		skeletons[sktIdx] = in_skeletons[sktIdx];
		processSkeleton(skeletons[sktIdx], defRig, subdivisions);
	}

	return true;
}


bool AirRig::bindLoadedRigToScene(Modelo* in_model, vector<skeleton*>& in_skeletons)
{
	// bind model
	bindModel(in_model);

	//bind skeletons
	skeletons.resize(in_skeletons.size());
	for(int i = 0; i< in_skeletons.size(); i++)
		skeletons[i] = in_skeletons[i];

	// get joint info
	
	map<string, joint*> allJoints;
	for(int i = 0; i< in_skeletons.size(); i++)
	{
		for(int idJoint = 0; idJoint < skeletons[i]->joints.size(); idJoint++)
		{
			allJoints[skeletons[i]->joints[idJoint]->sName] = skeletons[i]->joints[idJoint];
		}
	}
	
	map<int, int >& groupRel = serializedData->defGroupsRelation;
	map<int, int >& nodeRel = serializedData->defNodesRelation;

	// bind Groups to joints -> to remove???
	//map<int, int > nodeCorrespondence;
	for(int defgroupIdx = 0; defgroupIdx< defRig.defGroups.size(); defgroupIdx++)
	{
		// get correspondence
		//nodeCorrespondence[defRig.defGroups[defgroupIdx]->sNode->nodeId] = defRig.defGroups[defgroupIdx]->nodeId;
		defRig.defGroupsRef[defRig.defGroups[defgroupIdx]->nodeId] = defRig.defGroups[defgroupIdx];

		// bind to the transformation
		joint* jt = NULL;
		string jtName = defRig.defGroups[defgroupIdx]->serializedData->sJointName;
		defRig.defGroups[defgroupIdx]->transformation = allJoints[jtName];
		if(!defRig.defGroups[defgroupIdx]->transformation) 
			return false;

		delete defRig.defGroups[defgroupIdx]->serializedData;
		defRig.defGroups[defgroupIdx]->serializedData = NULL;

		// Updating deformers ids...
		for(int defIdx = 0; defIdx < defRig.defGroups[defgroupIdx]->deformers.size(); defIdx++)
		{
			DefNode& dnode = defRig.defGroups[defgroupIdx]->deformers[defIdx];
			dnode.boneId = groupRel[dnode.boneId];
			dnode.childBoneId = groupRel[dnode.childBoneId];

			defRig.defNodesRef[dnode.nodeId] = &dnode;

			defRig.deformers.push_back(&dnode);
		}

		// get correspondence
		//for(int defIdx = 0; defIdx< defRig.defGroups[defgroupIdx]->deformers.size(); defIdx++)
		//{
		//	groupRel[defRig.defGroups[defgroupIdx]->deformers[defIdx].sNode->nodeId] = defRig.defGroups[defgroupIdx]->deformers[defIdx].nodeId;
		//}
		defRig.defGroupsRef[defRig.defGroups[defgroupIdx]->nodeId] =defRig.defGroups[defgroupIdx];
	}

	// Bind Constraint Relation
	for(int relationIdx = 0; relationIdx< defRig.relations.size(); relationIdx++)
	{
		int childIdx = groupRel[defRig.relations[relationIdx]->sConstraint->childId];
		int parentIdx = groupRel[defRig.relations[relationIdx]->sConstraint->parentId];

		defRig.relations[relationIdx]->child = defRig.defGroupsRef[childIdx];
		defRig.relations[relationIdx]->parent = defRig.defGroupsRef[parentIdx] ;
	}

	BuildGroupTree(defRig);

	// changing node references
	for(int pt = 0; pt < model->bind->pointData.size(); pt++)
	{
		PointData& pd = model->bind->pointData[pt];
		pd.segmentId = nodeRel[pd.segmentId];
		
		for(int inflIdx = 0; inflIdx < pd.influences.size(); inflIdx++)
		{
			pd.influences[inflIdx].label = groupRel[pd.influences[inflIdx].label];
		}
	}

	//rig->skinning.bindings[0] = vector<binding*> ();
	//for(int i = 0; i< rig->model->bindings.size(); i++)
	//	rig->skinning.bindings[0].push_back(rig->model->bindings[i]);

	skin->deformedModel = model;
	skin->originalModel = model->originalModel;
	skin->bind = model->bind;

	// Default initialization
	//skin->bind = new binding();

	return false;
}

using namespace std;

bool bindRigToScene(Modelo* model, vector<skeleton*>skeletons, AirRig* rig)
{
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

	rig->skin->deformedModel = rig->model;
	rig->skin->originalModel = rig->model->originalModel;

	// Default initialization
	rig->skin->bind = rig->model->bind;

	return false;
}
