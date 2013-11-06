#include "AirRig.h"
#include <DataStructures/scene.h>
#include <Computation\Segmentation.h>
#include <Computation\mvc_interiorDistances.h>
#include <DataStructures\InteriorDistancesData.h>

//////////////////
//   AIRRIG     //
//////////////////
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
            else SaveEmbeddings(*model, bindingFileName, ascii);
        }
    }

	return success;
}

// CONSTRUCTORES
// An empty rig is created from a model
AirRig::AirRig(Modelo& in_model)
{
	// We get the model
	model = &in_model;
	
	// Preprocesses like distances over the surface
	// could be done, maybe as a backgroud process.
	// The data path and files from the model need to be coherent
	getBDEmbedding(model);

}

// An default configured rig is created from a model and a set of skeletons
AirRig::AirRig(Modelo& in_model, vector<skeleton*>& skts)
{
	// We get the model
	model = &in_model;

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
}

AirRig::~AirRig()
{
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