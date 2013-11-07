#include "AirSkinning.h"

#include <DataStructures\AirRig.h>
#include <set>

using namespace Eigen;

void AirSkinning::cacheSkinning()
{
	// We are considering just one model
	// with just one binding.

	binding* bd = bindings[0][0];
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

void AirSkinning::getDeformerRestPositions()
{
	// By now, we take the joint positions as a rest poses, but
	// next will be taken some control prositions solved for every deformer or group.
	deformersRestPosition.clear();
	for(int grIdx = 0; grIdx< rig->defRig.defGroups.size(); grIdx++)
	{
		int id = rig->defRig.defGroups[grIdx]->nodeId;
		deformersRestPosition[id] = joint(id);
		deformersRestPosition[id].setWorldPosition(rig->defRig.defGroups[grIdx]->transformation->getWorldPosition());
		
		// Al no tener las posiciones relativas de or y orient, no se si funcionara bien, quizás tenemos que ir a las globales.
		deformersRestPosition[id].qrot = rig->defRig.defGroups[grIdx]->transformation->qrot;
		deformersRestPosition[id].qOrient = rig->defRig.defGroups[grIdx]->transformation->qOrient;
	}
}

void AirSkinning::computeDeformationsWithSW()
{
}

void AirSkinning::computeDeformations()
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

	for (int i = 0; i < deformedModels.size(); ++i) 
	{		
		// It's a bad way to ensure that we are deforming the right mdoel.
		if (!deformedModels[i]->shading->visible) return;

		Geometry *m = deformedModels[i];
		for (int j = 0; j < bindings[i].size(); ++j) 
		{			
			// loop through all bindings
			binding * b = bindings[i][j];

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

					Vector3d& restPosition = originalModels[i]->nodes[vertexID]->position;
					Vector3d restPos2(restPosition.x(), restPosition.y(), restPosition.z());

					Quaterniond apliedRotation;
					apliedRotation = jt->rotation;
					//apliedRotation = apliedRotation * 

					if(rig->defRig.defGroupsRef[skID]->relatedGroups.size() == 1)
					{
						double twist = 0;
						
						Vector3d axis = rig->defRig.defGroupsRef[skID]->relatedGroups[0]->transformation->translation - rig->defRig.defGroupsRef[skID]->transformation->translation;
						axis.normalize();

						Quaterniond localRotationChild =  rig->defRig.defGroupsRef[skID]->relatedGroups[0]->transformation->qOrient * 
														  rig->defRig.defGroupsRef[skID]->relatedGroups[0]->transformation->qrot;

						Quaterniond localRotationChildRest =  rig->defRig.defGroupsRef[skID]->relatedGroups[0]->transformation->restRot;

						Vector3d referenceRest = localRotationChildRest._transformVector(axis);
						Vector3d referenceCurr = localRotationChild._transformVector(axis);

						Quaterniond nonRollrotation;
						nonRollrotation.setFromTwoVectors(referenceRest, referenceCurr);
						
						Quaterniond twistExtraction = localRotationChild*localRotationChildRest.inverse()*nonRollrotation.inverse();
						
						Quaterniond twistRotation(twistExtraction.w(), axis.x(), axis.y(), axis.z());
						twistRotation.normalize();

						float secondWeight = data.secondInfluences[kk][0];

						apliedRotation = apliedRotation * twistExtraction;
					}

					Vector3d finalPos2 =  apliedRotation._transformVector(jt->rRotation.inverse()._transformVector(restPos2-jt->rTranslation)) + jt->translation;
					
					//Vector4f restPos(restPosition.X(), restPosition.Y(), restPosition.Z(), 1);
					//Vector4f finalPos =  jt->W * jt->iT * restPos;	

					float weight = data.influences[kk].weightValue;

					//finalPosition = finalPosition + Vector3d(finalPos(0), finalPos(1), finalPos(2)) * weight;

					finalPosition = finalPosition + Vector3d(finalPos2(0), finalPos2(1), finalPos2(2)) * weight;

					totalWeight += data.influences[kk].weightValue;
					
				}

				finalPosition = finalPosition / totalWeight;
				if (m->nodes[vertexID]->position != finalPosition) 
					updated = true;

				m->nodes[vertexID]->position = finalPosition;
			}

		}
		if (updated)
		{		
			m->computeNormals();
		}
	}

}