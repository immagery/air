#include "AirSkinning.h"

#include <DataStructures\AirRig.h>
#include <set>

#include <iostream>
#include <fstream>

using namespace Eigen;

AirSkinning::~AirSkinning()
{

}

void AirSkinning::cacheSkinning()
{
	// We are considering just one model
	// with just one binding.

	binding* bd = bind;
	weights.clear();
	weights.resize(bd->pointData.size());

	for(int ptIdx = 0; ptIdx< bd->pointData.size(); ptIdx++)
	{
		PointData& pd = bd->pointData[ptIdx];
		weights[ptIdx].resize(pd.influences.size());
		for(int wIdx = 0; wIdx< pd.influences.size(); wIdx++)
		{
			weights[ptIdx][wIdx] = pd.influences[wIdx];
		}
	}
}

void AirSkinning::getDeformerRestPositions(AirRig* rig)
{

	// By now, we take the joint positions as a rest poses, but
	// next will be taken some control prositions solved for every deformer or group.
	deformersRestPosition.clear();
	for(int grIdx = 0; grIdx< rig->defRig.defGroups.size(); grIdx++)
	{
		int id = rig->defRig.defGroups[grIdx]->nodeId;
		deformersRestPosition[id] = new joint(id);
		deformersRestPosition[id]->setWorldPosition(rig->defRig.defGroups[grIdx]->transformation->getWorldPosition());
		
		// Al no tener las posiciones relativas de or y orient, no se si funcionara bien, quizás tenemos que ir a las globales.
		deformersRestPosition[id]->qrot = rig->defRig.defGroups[grIdx]->transformation->qrot;
		deformersRestPosition[id]->qOrient = rig->defRig.defGroups[grIdx]->transformation->qOrient;
	}
}

void AirSkinning::saveBindingToFile (string path)
{

}

void AirSkinning::loadBindingForModel(Modelo *m, AirRig* rig) 
{

	// Done in previous pases
	/*
	printf("Loading binding data...");

	ifstream in(path.c_str(), ios::in);
    //FILE* file = fopen(path.c_str(), "r");
	
	if (!in.is_open())
	{
		printf("\nNo binding file found! Proceeding to load model without bindings\n");
		return;
	}

	binding* bd = bind = new binding();

	deformedModel = (Geometry*) m;
	originalModel = m->originalModel;

	int counter = 0;
	while (!in.eof()) {
        
		string line;
		getline (in,line);
		stringstream ss(line);
		string item;
		vector<string> list;
		while (std::getline(ss, item, ' ')) 
		{
			list.push_back(item);
		}

		//PointData point;
		//point.node = new GraphNode();
		//point.node->id = list.at(0).toInt();
		//printf("Read binding for vertex %d:", point.node->id);
		//vertices.insert(point.node->id);

		PointData& point = bd->pointData[counter];
		counter++;

        for (int i = 1; i < list.size(); i += 2) {
			int skID = -1;
			string jointName = list[i];

			// DEBUG: we now search always in the last added skeleton, does this always work?
			//for(int skIdx = 0; skIdx < skeletons.size(); skIdx++) { }
			skeleton* sk = skeletons[skeletons.size()-1];
			for (int j = 0; j < sk->joints.size(); ++j) {
				joint* jt = sk->joints[j];
				if (jt->sName == jointName) {
					skID = jt->nodeId;
					break;
				}
			}	
			
			if (skID == -1) {
				printf("Something bad happened: joint does not exist!\n");
				assert(false);
			}

			float weightValue = atof(list[i+1].c_str());
			weight w;
			w.label = skID;
			w.weightValue = weightValue;
			point.influences.push_back(w);
        }

		getline (in,line);
		stringstream ss2(line);
		vector<string> secondList;
		while (std::getline(ss2, item, ' ')) 
		{
			secondList.push_back(item);
		}

		point.secondInfluences.resize(point.influences.size());
		int paramCount = 0;
		for(int infl = 0; infl < point.influences.size(); infl++)
		{
			int secCount = atoi(secondList[paramCount].c_str());
			point.secondInfluences[infl].resize(secCount);
			paramCount++;

			for(int sInfl = 0; sInfl < secCount; sInfl++)
			{
				point.secondInfluences[infl][sInfl] = atof(secondList[paramCount].c_str());
				paramCount++;
			}
		}
    }

	*/
}

void AirSkinning::computeDeformations(AirRig* rig)
{

	//if (deformersRestPosition.size() == 0) return;
	if(!rig) return;

	if (rig->defRig.defGroups.size() == 0) return;

	// Check if there's at least one dirty skeleton. In that case, proceed
	bool oneDirtySkeleton = false;
	for (int i = 0; i < rig->defRig.roots.size(); ++i) 
	{
		if (rig->defRig.roots[i]->transformation->dirtyFlag) 
		{
			oneDirtySkeleton = true;
			rig->defRig.roots[i]->transformation->computeWorldPos();
		}
	}
	if (!oneDirtySkeleton) return;
	
	bool updated = false;

	//for (int i = 0; i < deformedModels.size(); ++i) 
	{		
		// It's a bad way to ensure that we are deforming the right mdoel.
		if (!deformedModel->shading->visible) return;

		Geometry *m = deformedModel;

		// loop through all bindings
		binding * b = bind;

		for (int k = 0; k < b->pointData.size(); ++k) 
		{ 
			// and for each binding, loop over all its points
			PointData& data = b->pointData[k];
			GraphNode* node = data.node;
			int vertexID = node->id;

			Vector3d finalPosition (0,0,0);
			float totalWeight = 0;

			Vector3d rotDir; 
			for (int kk = 0; kk < data.influences.size(); ++kk) // and check all joints associated to them
			{   
				int skID = data.influences[kk].label;
				joint* jt = rig->defRig.defGroupsRef[skID]->transformation;

				Vector3d& restPosition = originalModel->nodes[vertexID]->position;
				Vector3d restPos2(restPosition.x(), restPosition.y(), restPosition.z());
					
				float currentWeight = data.influences[kk].weightValue;

				Vector3d finalPos2 =  jt->rotation._transformVector(jt->rRotation.inverse()._transformVector(restPos2-jt->rTranslation)) + jt->translation;
				finalPosition = finalPosition + Vector3d(finalPos2(0), finalPos2(1), finalPos2(2)) * currentWeight;

				totalWeight += data.influences[kk].weightValue;
					
			}

			finalPosition = finalPosition / totalWeight;
			if (m->nodes[vertexID]->position != finalPosition) 
				updated = true;

			m->nodes[vertexID]->position = finalPosition;
		}

		if (updated)
		{		
			m->computeNormals();
		}
	}
}

float isOverItsDefGroup(DefGroup* group, int nodeId)
{
	for(int defIdx = 0; defIdx < group->deformers.size(); defIdx++)
	{
		if(group->deformers[defIdx].nodeId = nodeId)
			return 1.0;
	}

	return 0.0;
}

bool thereIsInfluenceOf(vector<weight>& influences, int idInfl)
{
	for(int i = 0; i< influences.size(); i++)
	{
		if(influences[i].label == idInfl)
			return true;
	}

	return false;
}

double reScaleTwist(double twist, double ini, double fin)
{
	double newTwist = twist;
	ini = ini*0.5;
	fin = fin*0.5+0.5;
	if(twist <= ini ) newTwist = 0;
	else if(twist >= fin ) newTwist = 1.0;
	else
	{
		newTwist = (twist-ini)/(fin-ini); 
	}

	return newTwist;
}

void AirSkinning::computeDeformationsWithSW(AirRig* rig)
{
	//if (deformersRestPosition.size() == 0) return;
	if(!rig) return;

	if (rig->defRig.defGroups.size() == 0) return;

	// Check if there's at least one dirty skeleton. In that case, proceed
	bool oneDirtySkeleton = false;
	for (int i = 0; i < rig->defRig.roots.size(); ++i) 
	{
		if (rig->defRig.roots[i]->transformation->dirtyFlag) 
		{
			oneDirtySkeleton = true;
			rig->defRig.roots[i]->transformation->computeWorldPos();
		}
	}
	if (!oneDirtySkeleton) return;
	
	bool updated = false;

	//for (int i = 0; i < deformedModels.size(); ++i) 
	{		
		// It's a bad way to ensure that we are deforming the right mdoel.
		if (!deformedModel->shading->visible) return;

		Geometry *m = deformedModel;

		// loop through all bindings
		binding * b = bind;

		for (int k = 0; k < b->pointData.size(); ++k) 
		{ 
			// and for each binding, loop over all its points
			PointData& data = b->pointData[k];
			GraphNode* node = data.node;
			int vertexID = node->id;

			Vector3d finalPosition (0,0,0);
			float totalWeight = 0;

			Vector3d rotDir; 
			for (int kk = 0; kk < data.influences.size(); ++kk) // and check all joints associated to them
			{   
				int skID = data.influences[kk].label;
				//joint& jt = deformersRestPosition[skID];
				joint* jt = rig->defRig.defGroupsRef[skID]->transformation;

				if(!jt) continue;

				Vector3d& restPosition = originalModel->nodes[vertexID]->position;
				Vector3d restPos2(restPosition.x(), restPosition.y(), restPosition.z());

				Quaterniond apliedRotation = jt->rotation;			
				float currentWeight = data.influences[kk].weightValue;

				if(rig->defRig.defGroupsRef[skID]->relatedGroups.size() == 1 && rig->defRig.defGroupsRef[skID]->relatedGroups[0]->enableTwist)
				{

					if(data.secondInfluences[kk].size() == 0)
						continue;

					float secondWeight = data.secondInfluences[kk][0];

					// Twist Rescaling
					float iniTwist = rig->defRig.defGroupsRef[skID]->relatedGroups[0]->iniTwist;
					float finTwist = rig->defRig.defGroupsRef[skID]->relatedGroups[0]->finTwist;
					secondWeight = reScaleTwist(secondWeight, iniTwist , finTwist);

					Quaterniond childTwist = rig->defRig.defGroupsRef[skID]->relatedGroups[0]->transformation->twist;
					Quaterniond twist = rig->defRig.defGroupsRef[skID]->transformation->twist;

					float cutTwist = isOverItsDefGroup(rig->defRig.defGroupsRef[skID], data.segmentId);
					double cuttedWeight = currentWeight * (1.0-cutTwist);

					float secondCutTwist = 0.0;

					//if(!thereIsInfluenceOf(data.influences,skID))
					//	cuttedWeight = 0;

					Quaterniond twistRotation = Quaterniond::Identity().slerp(secondWeight, childTwist);

					/*
					//Quaterniond unTwist = Quaterniond::Identity().slerp(cuttedWeight, twist);
					//Quaterniond unTwist = Quaterniond::Identity();

					//if(jt->father)
					//	apliedRotation = jt->father->rotation*unTwist.inverse()*jt->qOrient*twistRotation*jt->qrot;
					//else
					//	apliedRotation = unTwist.inverse()*jt->qOrient*twistRotation*jt->qrot;
					*/

					if(jt->father)
						apliedRotation = jt->father->rotation*jt->qOrient*jt->qrot*twistRotation;
					else
						apliedRotation = jt->qOrient*jt->qrot*twistRotation;

				}

				Vector3d finalPos2 =  apliedRotation._transformVector(jt->rRotation.inverse()._transformVector(restPos2-jt->rTranslation)) + jt->translation;
				finalPosition = finalPosition + Vector3d(finalPos2(0), finalPos2(1), finalPos2(2)) * currentWeight;

				totalWeight += data.influences[kk].weightValue;
					
			}

			finalPosition = finalPosition / totalWeight;
			if (m->nodes[vertexID]->position != finalPosition) 
				updated = true;

			m->nodes[vertexID]->position = finalPosition;
		}
		if (updated)
		{		
			m->computeNormals();
		}
	}

}