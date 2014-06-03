#include "defGroup.h"

#include <Render/defGorupRender.h>
#include <DataStructures/scene.h>

//////////////////
//   DEFGROUP   //
//////////////////
DefGroup::DefGroup(int nodeId) : object(nodeId)
{
	transformation = NULL;
	rigTransform = NULL;
	smoothingPasses = default_SMOOTHING_PASES;
	smoothPropagationRatio = default_SMOOTHING_RATIO;
	subdivisionRatio = default_SUBDIVISION_RATIO;
	expansion = default_EXPANSION;

	iniTwist = default_INI_TWIST;
	finTwist = default_END_TWIST;
	enableTwist = true;
	smoothTwist = false;

	localSmooth = false;

	iam = DEFGROUP_NODE;

	shading = new DefGroupRender(this);

	parentType = 0;
	type = DEF_STICK;

	// To perform bulge effect
	bulgeEffect = false;

	references = NULL;
}


DefGroup::DefGroup(int nodeId, joint* jt) : object(nodeId)
{
	transformation = jt;

	smoothingPasses = default_SMOOTHING_PASES;
	smoothPropagationRatio = default_SMOOTHING_RATIO;
	subdivisionRatio = default_SUBDIVISION_RATIO;
	expansion = default_EXPANSION;

	iniTwist = default_INI_TWIST;
	finTwist = default_END_TWIST;
	enableTwist = true;

	localSmooth = false;

	iam = DEFGROUP_NODE;

	shading = new DefGroupRender(this);

	parentType = 0;

	type = DEF_STICK;

	// To perform bulge effect
	bulgeEffect = false;
}

bool DefGroup::selectRec(bool bToogle)
{
	shading->selected = bToogle;
	for(unsigned int i = 0; i< relatedGroups.size(); i++)
		relatedGroups[i]->selectRec(bToogle);

	return true;
}

bool DefGroup::update()
{
    if(!dirtyFlag)
        return true;
    else
	{

		if(dirtyTransformation)
		{
			// En este caso tengo que actualizar tambien el padre.
			for(int i = 0; i< dependentGroups.size(); i++)
			{
				updateDefNodesFromStick(dependentGroups[i]);
			}
		}

		if(dependentGroups.size() > 0)
			computeWorldPosRec(this, dependentGroups[0]);
		else
			computeWorldPosRec(this, NULL);

		// Reconstruir el esquema de deformadores.
		if(type == DEF_STICK)
		{
			updateDefNodesFromStick(this);
		}

		// El defGroup queda limpio, pero los defNodes
		// quedan sucios hasta que alguien los recompute.
		dirtyFlag = false;
        return true;
	}
}

bool DefGroup::select(bool bToogle, unsigned int id)
{
	bToogle &= ((unsigned int)id == nodeId);

	shading->selected = bToogle;
	for(unsigned int i = 0; i< relatedGroups.size(); i++)
		relatedGroups[i]->selectRec(bToogle);

	return true;
}

void DefGroup::computeWorldPosRec(DefGroup* dg, DefGroup* fatherDg)
{
	Eigen::Matrix3d rotationMatrix;

	joint* jt = dg->transformation;
	joint* father = NULL;

	if(fatherDg != NULL) 
		father = fatherDg->transformation;

	if(!fatherDg)
	{
		jt->translation = jt->pos;
		jt->rotation =  jt->qOrient * jt->qrot;

		rotationMatrix = jt->rotation.toRotationMatrix();

		jt->twist = Quaterniond::Identity();
		jt->parentRot = Eigen::Quaterniond::Identity();
	}
	else
	{
		jt->translation = father->translation + 
						  father->rotation._transformVector(jt->pos);
		
		jt->rotation =  father->rotation * jt->qOrient * jt->qrot;

		rotationMatrix = ( jt->qOrient * jt->qrot).toRotationMatrix();

		// TwistComputation:
		Vector3d axis = jt->rTranslation - father->rTranslation;
		axis.normalize();

		Quaterniond localRotationChild =  jt->qOrient * jt->qrot;
		Quaterniond localRotationChildRest =  jt->restRot;

		Vector3d tempRestRot = father->rRotation.inverse()._transformVector(axis);

		Vector3d referenceRest = localRotationChildRest._transformVector(tempRestRot);
		Vector3d referenceCurr = localRotationChild._transformVector(tempRestRot);

		Quaterniond nonRollrotation;
		nonRollrotation.setFromTwoVectors(referenceRest, referenceCurr);
		
		// Ejes Locales
		jt->twist = localRotationChild*localRotationChildRest.inverse()*nonRollrotation.inverse();

		Vector3d quatAxis(jt->twist.x(),jt->twist.y(),jt->twist.z());
		quatAxis = (localRotationChild).inverse()._transformVector(quatAxis);
		//quatAxis = father->qOrient._transformVector(quatAxis);
		//quatAxis = (father->qOrient * father->qrot).inverse()._transformVector(quatAxis);
	
		jt->twist.x() = quatAxis.x();
		jt->twist.y() = quatAxis.y();
		jt->twist.z() = quatAxis.z();

		Vector3d referencePoint = father->rotation.inverse()._transformVector(jt->translation - father->translation);
		Quaterniond redir;
		jt->parentRot = father->rotation * redir.setFromTwoVectors(Vector3d(1,0,0),referencePoint);	

	}
	
	//Eigen::Matrix3d rotationMatrix;
	//rotationMatrix = jt->rRotation.toRotationMatrix();

	Eigen::Matrix4f transformMatrix2;
	transformMatrix2 << rotationMatrix(0,0) , rotationMatrix(0,1) , rotationMatrix(0,2), jt->pos[0],
					    rotationMatrix(1,0) , rotationMatrix(1,1) , rotationMatrix(1,2), jt->pos[1],
					    rotationMatrix(2,0) , rotationMatrix(2,1) , rotationMatrix(2,2), jt->pos[2],
						0.0,				0.0,				0.0,			1.0;

	transformMatrix2.transposeInPlace();
	jt->world = transformMatrix2;

	jt->worldPosition = jt->translation;
	jt->W = transformMatrix2.transpose();
	

	for(unsigned int i = 0; i< dg->relatedGroups.size(); i++)
    {
        computeWorldPosRec(dg->relatedGroups[i], dg);
    }

	return;

}


void DefGroup::addTranslation(double tx, double ty, double tz)
{
}

void DefGroup::setTranslation(double tx, double ty, double tz, bool local)
{
	if(local)
	{
		// Como se hacia antes.. a pinon, pero se trata de restaurar la jerarquia.
		/*
		// The in quaternion is expresed in local coords.
		// we can assign it directly.
		transformation->pos = Vector3d(tx, ty, tz);

		if(AirRig::mode == MODE_RIG || AirRig::mode == MODE_TEST || AirRig::mode == MODE_CREATE)
				transformation->restPos = transformation->pos;

	    // parent and all the childs
		if(AirRig::mode == MODE_RIG || AirRig::mode == MODE_CREATE)
			dirtyByTransformation(false);

		*/

		Vector3d increment;
		if(dependentGroups.size() == 0)
		{
			// The in quaternion is expressed in global coords.
			// We need to do transformation to bring it to local coords.
			Vector3d despl =  Vector3d(tx, ty, tz) - transformation->pos;
			transformation->pos = Vector3d(tx, ty, tz);
			increment = despl;
		}
		else
		{
			// The in quaternion is expressed in global coords.
			// We need to do transformation to bring it to local coords.
			Vector3d despl = Vector3d(tx, ty, tz)-dependentGroups[0]->getTranslation(false);
			despl = dependentGroups[0]->transformation->rotation.inverse()._transformVector(despl);
			increment = despl - transformation->pos;

			//despl = transformation->qOrient.inverse()._transformVector(despl);
			transformation->pos = despl;
		}

		for(int i = 0; i < relatedGroups.size(); i++)
		{
			// tengo que calcular algo mas... a ver que toca
			Vector3d tempPt = relatedGroups[i]->transformation->pos;
			tempPt = (transformation->qOrient*transformation->qrot).inverse()._transformVector(tempPt);
			tempPt -= increment;
			tempPt = (transformation->qOrient*transformation->qrot)._transformVector(tempPt);
			relatedGroups[i]->transformation->pos = tempPt; 
		}

		if(AirRig::mode == MODE_RIG || AirRig::mode == MODE_TEST || AirRig::mode == MODE_CREATE)
			transformation->restPos = transformation->pos;

		if(AirRig::mode == MODE_RIG || AirRig::mode == MODE_CREATE)
			dirtyByTransformation(true, false);

		//printf("Modificacion en local\n");

	}
	else
	{
		//printf("Modificacion jerarquica\n");

		if(dependentGroups.size() == 0)
		{
			// The in quaternion is expressed in global coords.
			// We need to do transformation to bring it to local coords.
			transformation->pos = Vector3d(tx, ty, tz);
		}
		else
		{
			// The in quaternion is expressed in global coords.
			// We need to do transformation to bring it to local coords.
			Vector3d despl = Vector3d(tx, ty, tz)-dependentGroups[0]->getTranslation(false);

			despl = dependentGroups[0]->transformation->rotation.inverse()._transformVector(despl);
			//despl = transformation->qOrient.inverse()._transformVector(despl);
			transformation->pos = despl;
		}

		
		if(AirRig::mode == MODE_RIG || AirRig::mode == MODE_TEST || AirRig::mode == MODE_CREATE)
			transformation->restPos = transformation->pos;

		if(AirRig::mode == MODE_RIG || AirRig::mode == MODE_CREATE)
			dirtyByTransformation(true, true);

	}
}

void DefGroup::addRotation(double rx, double ry, double rz)
{

}

void DefGroup::addRotation(Eigen::Quaternion<double> q, bool local)
{

}

void DefGroup::setRotation(Eigen::Quaternion<double> q, bool local)
{
	if(false /*local*/)
	{
		// The in quaternion is expresed in local coords.
		// we can assign it directly.
		q.normalize();

		// The defgraph is in this rig
		transformation->setRotation(q);

		if(AirRig::mode == MODE_RIG || AirRig::mode == MODE_TEST || AirRig::mode == MODE_CREATE)
			transformation->restRot = transformation->qrot;

		 // not parent but all the childs
		if(AirRig::mode == MODE_RIG || AirRig::mode == MODE_CREATE)
			dirtyByTransformation(true, false);
	}
	else
	{
		// The in quaternion is expressed in global coords.
		// We need to do transformation to bring it to local coords.
		q.normalize();
		if(dependentGroups.size() == 0)
		{
			// Is the root
			// The defgraph is in this rig
			transformation->setRotation(transformation->qOrient.inverse()*q);

			if(AirRig::mode == MODE_RIG || AirRig::mode == MODE_TEST || AirRig::mode == MODE_CREATE)
				transformation->restRot = transformation->qrot;

			// not parent but all the childs
			if(AirRig::mode == MODE_RIG || AirRig::mode == MODE_CREATE)
				dirtyByTransformation(true);
		}
		else
		{
			Quaterniond newq = transformation->qOrient.inverse()*dependentGroups[0]->getRotation(false).inverse()*q;
			newq.normalize();
			transformation->setRotation(newq);

			if(AirRig::mode == MODE_RIG || AirRig::mode == MODE_TEST || AirRig::mode == MODE_CREATE)
				transformation->restRot = transformation->qrot;

			// not parent but all the childs
			if(AirRig::mode == MODE_RIG || AirRig::mode == MODE_CREATE)
				dirtyByTransformation(true);
		}
	}
}

Quaterniond DefGroup::getRotation(bool local)
{
	Quaterniond rot;

	if(local)
		return transformation->qrot;
	else
		return transformation->rotation;

	return rot;
}

Quaterniond DefGroup::getRestRotation(bool local)
{
	Quaterniond rot;

	if(local)
		return transformation->qrot;
	else
		return transformation->rRotation;

	return rot;
}

Vector3d DefGroup::getRestTranslation(bool local)
{
	if(local)
		return transformation->pos;
	else
		return transformation->rTranslation;

	Vector3d pos(0,0,0);
	return pos;
}


Vector3d DefGroup::getTranslation(bool local)
{
	if(local)
		return transformation->pos;
	else
		return transformation->translation;

	Vector3d pos(0,0,0);
	return pos;
}

bool DefGroup::saveToFile(FILE* fout)
{
	if(!fout)
	{
		printf("There is no file to print!\n [AIR_RIG]");
		return false;
	}

	node::saveToFile(fout);

	fprintf(fout, "%d ", deformers.size()); fflush(fout);
	if(transformation->sName.length()>0)
	{
		fprintf(fout, "%s ", transformation->sName.c_str()); 
		fflush(fout);
	}
	else
	{
		fprintf(fout, "AirRig_noNameJoint_\n", transformation->sName.c_str()); 
		fflush(fout);
	}
	fprintf(fout, "\n%f %f %d %f ", subdivisionRatio, expansion, smoothingPasses,smoothPropagationRatio); fflush(fout);
	fprintf(fout, "%d\n", (int)type); fflush(fout);

	for(int i = 0; i< deformers.size(); i++)
		deformers[i].saveToFile(fout);

	// For computational pourposes.
	//vector<DefGroup*> relatedGroups;

	return true;
}

void DefGroup::computeRestPos(DefGroup* dg, DefGroup* father)
{
	Eigen::Matrix3d rotationMatrix;
	joint* jt = dg->transformation;
	 
	if(!father)
	{
		jt->rTranslation = jt->pos;
		jt->rRotation = jt->qOrient * jt->qrot;

		rotationMatrix = jt->rRotation.toRotationMatrix();

		jt->rTwist = Quaterniond::Identity();
	}
	else
	{
		joint* fatherJt = father->transformation;
		jt->rTranslation = fatherJt->rTranslation + 
						   fatherJt->rRotation._transformVector(jt->pos);
		
		jt->rRotation =  fatherJt->rRotation * jt->qOrient * jt->qrot;

		rotationMatrix = ( jt->qOrient * jt->qrot ).toRotationMatrix();
		//rotationMatrix = jt->rRotation.toRotationMatrix();

		// TwistComputation:
		Vector3d axis = jt->translation - fatherJt->translation;
		axis.normalize();

		Vector3d tempRestRot = fatherJt->rRotation._transformVector(axis);

		///if(parentType > 0)
		//	tempRestRot = axis;

		Quaterniond localRotationChild =  jt->qOrient * jt->qrot;
		Quaterniond localRotationChildRest =  jt->restRot;
		Vector3d referenceRest = localRotationChildRest._transformVector(tempRestRot);
		Vector3d referenceCurr = localRotationChild._transformVector(tempRestRot);

		Quaterniond nonRollrotation;
		nonRollrotation.setFromTwoVectors(referenceRest, referenceCurr);
		
		// Ejes Locales
		jt->rTwist = localRotationChild*localRotationChildRest.inverse()*nonRollrotation.inverse();
	}

	jt->restPos = jt->pos;
	//jt->restRot = jt->qOrient * jt->qrot;
	jt->restRot = jt->qrot;
	jt->restOrient = jt->qOrient;

	//Eigen::Matrix3d rotationMatrix;
	//rotationMatrix = jt->rRotation.toRotationMatrix();

	Eigen::Matrix4f transformMatrix2;
	transformMatrix2 << rotationMatrix(0,0) , rotationMatrix(0,1) , rotationMatrix(0,2), jt->pos[0],
					   rotationMatrix(1,0) , rotationMatrix(1,1) , rotationMatrix(1,2), jt->pos[1],
					   rotationMatrix(2,0) , rotationMatrix(2,1) , rotationMatrix(2,2), jt->pos[2],
						0.0,				0.0,				0.0,			1.0;

	transformMatrix2.transposeInPlace();
	jt->iT = transformMatrix2.inverse().transpose();
	
	jt->worldPosition = jt->rTranslation;

	for(unsigned int i = 0; i< dg->relatedGroups.size(); i++)
    {
        computeRestPos(dg->relatedGroups[i], dg);
    }

	return;
}



void DefGroup::computeWorldPos(DefGroup* dg, DefGroup* father)
{
	Eigen::Matrix3d rotationMatrix;
	joint* jt = dg->transformation;

	if(!father)
	{
		jt->translation = jt->pos;
		jt->rotation =  jt->qOrient * jt->qrot;

		rotationMatrix = jt->rotation.toRotationMatrix();

		jt->twist = Quaterniond::Identity();
		jt->parentRot = Eigen::Quaterniond::Identity();
	}
	else
	{
		joint* fatherJt = father->transformation;
		jt->translation = fatherJt->translation + 
						  fatherJt->rotation._transformVector(jt->pos);
		
		if(parentType == 0)
			jt->rotation =  fatherJt->rotation * jt->qOrient * jt->qrot;
		else
			jt->rotation =  fatherJt->rRotation * jt->qOrient * jt->qrot;

		rotationMatrix = (jt->qOrient * jt->qrot).toRotationMatrix();

		// TwistComputation:
		Vector3d axis = jt->rTranslation - fatherJt->rTranslation;
		axis.normalize();

		Quaterniond localRotationChild =  jt->qOrient * jt->qrot;
		Quaterniond localRotationChildRest =  jt->restRot;

		if(parentType > 0)
		{
			//Quaterniond noFather = fatherJt->rotation*fatherJt->rRotation.inverse();
			//localRotationChild = noFather*jt->qOrient * jt->qrot;
		}

		Vector3d tempRestRot = fatherJt->rRotation.inverse()._transformVector(axis);

		Vector3d referenceRest = localRotationChildRest._transformVector(tempRestRot);
		Vector3d referenceCurr = localRotationChild._transformVector(tempRestRot);

		Quaterniond nonRollrotation;
		nonRollrotation.setFromTwoVectors(referenceRest, referenceCurr);
		
		// Ejes Locales
		jt->twist = localRotationChild*localRotationChildRest.inverse()*nonRollrotation.inverse();

		if(parentType > 0)
		{
			/*
			// TwistComputation:
			Vector3d fatherAxis = jt->translation - fatherJt->translation;
			Vector3d childAxis = jt->rTranslation - fatherJt->rTranslation;
			fatherAxis.normalize();
			childAxis.normalize();

			Quaterniond localRotationFather =  fatherJt->rotation;
			Quaterniond localRotationChild =  jt->rotation;

			Vector3d referenceRest = fatherJt->rotation._transformVector(fatherAxis);
			Vector3d referenceCurr = fatherJt->rotation._transformVector(childAxis);

			Quaterniond nonRollrotation;
			nonRollrotation.setFromTwoVectors(referenceRest, referenceCurr);

			Quaterniond dif = localRotationChild*fatherJt->rRotation.inverse();

			Quaterniond retwist = dif*nonRollrotation.inverse();

			jt->twist *= retwist;
			*/
		}

		Vector3d quatAxis(jt->twist.x(),jt->twist.y(),jt->twist.z());

		if(parentType == 0)
		{
			quatAxis = (localRotationChild).inverse()._transformVector(quatAxis);
		}
		else
			quatAxis = (localRotationChild).inverse()._transformVector(quatAxis);
		//quatAxis = father->qOrient._transformVector(quatAxis);
		//quatAxis = (father->qOrient * father->qrot).inverse()._transformVector(quatAxis);
	
		jt->twist.x() = quatAxis.x();
		jt->twist.y() = quatAxis.y();
		jt->twist.z() = quatAxis.z();

		Vector3d referencePoint = fatherJt->rotation.inverse()._transformVector(jt->translation - fatherJt->translation);
		Quaterniond redir;
		jt->parentRot = fatherJt->rotation * redir.setFromTwoVectors(Vector3d(1,0,0),referencePoint);	

	}
	
	//Eigen::Matrix3d rotationMatrix;
	//rotationMatrix = jt->rRotation.toRotationMatrix();

	Eigen::Matrix4f transformMatrix2;
	transformMatrix2 << rotationMatrix(0,0) , rotationMatrix(0,1) , rotationMatrix(0,2), jt->pos[0],
					    rotationMatrix(1,0) , rotationMatrix(1,1) , rotationMatrix(1,2), jt->pos[1],
					    rotationMatrix(2,0) , rotationMatrix(2,1) , rotationMatrix(2,2), jt->pos[2],
						0.0,				0.0,				0.0,			1.0;

	transformMatrix2.transposeInPlace();
	jt->world = transformMatrix2;

	jt->worldPosition = jt->translation;
	jt->W = transformMatrix2.transpose();
	

	for(unsigned int i = 0; i< dg->relatedGroups.size(); i++)
    {
        computeWorldPos(dg->relatedGroups[i], dg);
    }

	return;
}

void DefGroup::restorePoses(DefGroup* dg, DefGroup* father)
{
	joint* jt = dg->transformation;

	jt->qrot = jt->restRot;
	jt->pos = jt->restPos;
	jt->qOrient = jt->restOrient;

	jt->translation = jt->rTranslation;
	jt->rotation = jt->rRotation;
	jt->twist = jt->rTwist;

	for(int jtChild = 0; jtChild < dg->relatedGroups.size(); jtChild++)
	{
		dg->relatedGroups[jtChild]->restorePoses(dg->relatedGroups[jtChild], dg);
	}
}

// Loads data from this file, it is important to bind
// this data with the model and skeleton after this function.
bool DefGroup::loadFromFile(ifstream& in, airRigSerialization* sData)
{
	node::loadFromFile(in);

	string line;
	vector<string> elems;

	getline (in , line);
    split(line, ' ', elems);

	serializedData = new defGroupSerialization();

	int deformersSize = atoi(elems[0].c_str());
	serializedData->sJointName = elems[1];

	getline (in , line);
    split(line, ' ', elems);

	subdivisionRatio = atof(elems[0].c_str());
	expansion = atof(elems[1].c_str());
	smoothingPasses = atoi(elems[2].c_str());
	smoothPropagationRatio = atof(elems[3].c_str());
	type = (DefGroupType)atoi(elems[4].c_str());

	sData->defGroupsRelation[sNode->nodeId] = nodeId;

	deformers.resize(deformersSize);
	for(int i = 0; i< deformers.size(); i++)
	{
		deformers[i].nodeId = scene::getNewId(T_DEFNODE);

		if(!deformers[i].loadFromFile(in)) return false;
		
		sData->defNodesRelation[deformers[i].sNode->nodeId] = deformers[i].nodeId;
		delete deformers[i].sNode;
	}

	return true;
}

void DefGroup::setRotation(double rx, double ry, double rz, bool radians)
{
	if(!transformation) return;

	Eigen::Quaterniond q[3];
	double angles[3];

	angles[0] = rx;
	angles[1] = ry;
	angles[2] = rz;

	if(radians)
	{
		// Convert to degrees
		for(int i = 0; i< 3; i++)
			angles[i] = angles[i]*360/(M_PI*2);
	}

	// Rotation over each axis
	for(int i = 0; i< 3; i++)
		getAxisRotationQuaternion(q[i], i, angles[i]);

	// Concatenate all the values in X-Y-Z order
	Eigen::Quaterniond qrotAux =  q[2] * q[1] * q[0];

	// TOFIX: remove eigen rotations
	transformation->qrot = qrotAux; // Quaternion<double>(qrotAux.w(), qrotAux.x(),qrotAux.y(),qrotAux.z());

	int test = 0;
}


bool DefGroup::dirtyByTransformation(bool alsoFather, bool hierarchically)
{
	// we set dirty all the deformers form the parents that goes to this node
	if(alsoFather)
	{
		for(int dp = 0; dp < dependentGroups.size(); dp++)
		{
			//Vector3d dir = (transformation->translation - dependentGroups[dp]->transformation->translation).normalized();
			for(int dn = 0; dn < dependentGroups[dp]->deformers.size(); dn++)
			{
				if(dependentGroups[dp]->deformers[dn].childBoneId == nodeId)
				{
					// Rotate de defGroup to repositionate.
					/*Vector3d dnPos = dependentGroups[dp]->deformers[dn].pos - 
									 dependentGroups[dp]->transformation->translation;
					Quaterniond q;
					q.setFromTwoVectors(dnPos.normalized(), dir);
					dnPos = q._transformVector(dnPos);
					dependentGroups[dp]->deformers[dn].pos = dependentGroups[dp]->transformation->translation + dnPos;
					dependentGroups[dp]->deformers[dn].relPos = 
									dependentGroups[dp]->transformation->rotation.inverse()._transformVector(dnPos);*/

					dependentGroups[dp]->deformers[dn].dirtyFlag = true;
					dependentGroups[dp]->dirtyTransformation = true;
				}
			}
		}
	}

	dirtyTransformation = true;

	// we set dirty all the deformers of this node
	for(int dn = 0; dn < deformers.size(); dn++)
	{
		/*
		int childId = deformers[dn].childBoneId;

		if(childId < 0) continue;

		Vector3d dirToChild = ((*references)[childId]->transformation->translation -  transformation->translation).normalized();

		// Rotate de defGroup to repositionate.
		Vector3d dnPos = transformation->translation - deformers[dn].pos;

		Quaterniond q;
		q.setFromTwoVectors(dnPos.normalized(), dirToChild);

		dnPos = q._transformVector(dnPos);
		deformers[dn].pos = transformation->translation + dnPos;
		deformers[dn].relPos = transformation->rotation.inverse()._transformVector(dnPos);
		*/

		deformers[dn].dirtyFlag = true;
		deformers[dn].segmentationDirtyFlag = true;
	}

	// Propagate over all the dependant relations
	if(hierarchically)
	{
		for(int i = 0; i< relatedGroups.size(); i++)
			relatedGroups[i]->dirtyByTransformation(true, hierarchically);
	}

	return false;
}

bool DefGroup::dirtyBySegmentation()
{
	// we set dirty all the deformers of this group
	for(int dn = 0; dn < deformers.size(); dn++)
		deformers[dn].segmentationDirtyFlag = true;

	return false;
}

bool DefGroup::dirtyBySmoothing()
{
	assert(false);
	return false;
}


bool DefGroup::propagateDirtyness()
{
    dirtyFlag = true;

	for(int i = 0; i< relatedGroups.size(); i++)
		relatedGroups[i]->propagateDirtyness();

    return true;
}
