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
		if(group->deformers[defIdx].nodeId == nodeId)
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
				DefGroup* currentGroup = rig->defRig.defGroupsRef[skID];

				joint* jt = currentGroup->transformation;
				if(!jt) continue;

				Vector3d& restPosition = originalModel->nodes[vertexID]->position;

				Quaterniond apliedRotation = jt->rotation;
				float currentWeight = data.influences[kk].weightValue;

				if(data.secondInfluences[kk].size() != 0)
				{
					// Tenemos pesos secundarios según este hueso
					vector<Quaterniond> inducedTwists;
					for(int childGroupIdx = 0; childGroupIdx< currentGroup->relatedGroups.size(); childGroupIdx++)
					{
						DefGroup* childGroup = currentGroup->relatedGroups[childGroupIdx];

						if(childGroup->enableTwist)
						{
							float secondWeight = data.secondInfluences[kk][childGroupIdx].alongBone;
							Quaterniond childTwist = childGroup->transformation->twist;

							// Twist Rescaling
							float iniTwist = childGroup->iniTwist;
							float finTwist = childGroup->finTwist;		

							secondWeight = reScaleTwist(secondWeight, iniTwist , finTwist);

							inducedTwists.push_back(Quaterniond::Identity().slerp(secondWeight, childTwist));
						}
						else
						{
							inducedTwists.push_back(Quaterniond::Identity());
						}
					}
					
					vector<Vector3d> inducedPoints(inducedTwists.size());
					for(int it = 0; it< inducedTwists.size(); it++)
					{
						if(jt->father)
							apliedRotation = jt->father->rotation*jt->qOrient*jt->qrot*inducedTwists[it];
						else
							apliedRotation = jt->qOrient*jt->qrot*inducedTwists[it];

						inducedPoints[it] = apliedRotation._transformVector(jt->rRotation.inverse()._transformVector(restPosition-jt->rTranslation)) + jt->translation;
					}

					double weight = 0;
					Vector3d finalPos2(0,0,0);
					for(int it = 0; it< inducedPoints.size(); it++)
					{
						finalPos2 += inducedPoints[it] * data.secondInfluences[kk][it].wideBone;
						weight += data.secondInfluences[kk][it].wideBone;
					}

					if(weight != 0) finalPos2 /= weight;

					/*
					//vector<Quaterniond*> otherTwists;
					if(inducedTwists.size() > 0)
					{
						twistRotation = inducedTwists[0];

						// Solo estaran los habilitados, es imposible o al menos eso parece
						// hacer una mezcla lógica ocn todos. 
						for(int twistIdx = 1; twistIdx < inducedTwists.size(); twistIdx++)
						{
							//if(currentGroup->relatedGroups[twistIdx]->nodeId != principalTwist)
							//{
								float wide = data.secondInfluences[kk][twistIdx].wideBone;
								//float influenceWeightValue = data.influences[kk].weightValue;
								twistRotation.slerp(wide, inducedTwists[twistIdx]);
							//}
						}
						

						// Buscar el principal.
						//int principalTwist = rig->defRig.defNodesRef[data.segmentId]->childBoneId;
							
						//for(int twistIdx = 0; twistIdx < inducedTwists.size(); twistIdx++)
						//{
						//	if(currentGroup->relatedGroups[twistIdx]->nodeId == principalTwist)
						//	{
						//		twistRotation = inducedTwists[twistIdx];
						//		break;
						//	}
							//float wide = data.secondInfluences[kk][twistIdx].wideBone;
							//float influenceWeightValue = data.influences[kk].weightValue;
							//twistRotation *= Quaterniond::Identity().slerp(wide*influenceWeightValue, inducedTwists[twistIdx]);
						//}

					}
					

					//if(jt->father)
					//	apliedRotation = jt->father->rotation*jt->qOrient*jt->qrot*twistRotation;
					//else
					//	apliedRotation = jt->qOrient*jt->qrot*twistRotation;
					*/

					finalPosition = finalPosition + finalPos2 * currentWeight;
					totalWeight += currentWeight;

				}
				else
				{
					Vector3d finalPos2 =  apliedRotation._transformVector(jt->rRotation.inverse()._transformVector(restPosition-jt->rTranslation)) + jt->translation;
					finalPosition = finalPosition + finalPos2 * currentWeight;

					totalWeight += data.influences[kk].weightValue;
				}					
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

void saveAirBinding(binding* bd, string fileName)
{
	printf("\nGuardando %s\n",fileName.c_str());
	FILE* fout = fopen(fileName.c_str(), "w");

	fprintf(fout, "%d\n", bd->pointData.size());
	for(int pt = 0; pt< bd->pointData.size(); pt++)
	{
		bd->pointData[pt].saveToFile(fout);
	}

	fclose(fout);
}

void loadAirBinding(binding* bd, string fileName)
{
	ifstream inFile;
	inFile.open(fileName.c_str());

	if(inFile.is_open())
	{
		int pointSize = 0;
		string str;
		getline(inFile, str);
		pointSize = atoi(str.c_str());
		assert(pointSize <= bd->pointData.size());
		for(int pt = 0; pt< bd->pointData.size(); pt++)
		{
			bd->pointData[pt].loadFromFile(inFile);
		}

		inFile.close();
	}
}