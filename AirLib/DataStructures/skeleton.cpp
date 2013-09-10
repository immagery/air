#include "skeleton.h"
#include "Scene.h"

#define VERBOSE false

#include <utils/util.h>
#include <render/skeletonRender.h>

// COW
#define subdivisionRatio_DEF 0.1
#define endChop_DEF 0.001

//TROLL
//#define subdivisionRatio_DEF 5.0
//#define endChop_DEF 0.01

#define ratioExpansion_DEF 0.7


void joint::initDefaults()
{
	father = NULL;
    orientJoint = Point3d(0,0,0);
    childs.clear();
    nodes.clear();
    
    iam = JOINT_NODE;

	embedding.clear();
	childVector.clear();

	deformerId = -1; // not initialized
	expansion = 1;
	smoothness = 1;

	shading = new JointRender(this);
}

joint::joint() : object()
{
	initDefaults();

	/*
    father = NULL;
    orientJoint = Point3d(0,0,0);
    childs.clear();
    nodes.clear();

    iam = JOINT_NODE;

	embedding.clear();
	childVector.clear();

	deformerId = -1; // not initialized
	expansion = 1;
	smoothness = 1;

	shading = new JointRender(this);
	*/
}

joint::joint(unsigned int nodeId) : object(nodeId)
{
	initDefaults();

	/*
	iam = JOINT_NODE;
	
	father = NULL;
    orientJoint = Point3d(0,0,0);
    childs.clear();
    nodes.clear();

    expansion = 1;
	smoothness = 1;

	deformerId = -1; // not initialized
	
	embedding.clear();
	childVector.clear();

	shading = new JointRender(this);
	*/
}

joint::joint(joint* _father) : object()
{
	initDefaults();

    father = _father;
    
	/*
	orientJoint = Point3d(0,0,0);
    //JointSkeletonId = 0;
    childs.clear();
    nodes.clear();

    expansion = 1;
	smoothness = 1;

	deformerId = -1; // not initialized

    iam = JOINT_NODE;

	embedding.clear();
	childVector.clear();

	shading = new JointRender(this);
	*/
}

joint::joint(joint* _father, unsigned int nodeId) : object(nodeId)
{
    father = _father;
    orientJoint = Point3d(0,0,0);
    //JointSkeletonId = 0;
    childs.clear();
    nodes.clear();
    expansion = 1;
	smoothness = 1;
    iam = JOINT_NODE;

	embedding.clear();
	childVector.clear();

	shading = new JointRender(this);
}

joint::~joint()
{
    for(unsigned int i = 0; i< childs.size(); i++)
    {
        delete childs[i];
    }

    childs.clear();
    nodes.clear();

    father = NULL;

    orientJoint = Point3d(0,0,0);
}

void joint::setFather(joint* f)
{
    father = f;
    //printf("SetFather\n"); fflush(0);
}

joint* joint::getFather()
{
    return father;
}

void joint::pushChild(joint* child)
{
    childs.push_back(child);
   // printf("PushChild\n"); fflush(0);
}

int joint::getChildCount()
{
    //printf("GetChildCount\n"); fflush(0);
    return childs.size();
}

joint* joint::getChild(int id)
{
    if(id < 0 || id >= (int)childs.size())
    {
        printf("Error!. Acceso a array de joints fuera de rango.");
        return NULL;
    }

    //printf("Get Child\n"); fflush(0);

    return childs[id];
}

void joint::setWorldPosition(Point3d pos)
{
    //printf("setWorldPosition\n"); fflush(0);
    worldPosition = pos;
}

Point3d joint::getWorldPosition()
{
    return worldPosition;
}


bool joint::getBoundingBox(Point3d& minAuxPt,Point3d& maxAuxPt)
{
    // Devuelve su longitud más es máximo de todo lo que cuelga
    maxAuxPt.X() = 0;

    double maxLongTemp = 0;
    for(unsigned int i = 0; i< childs.size(); i++)
    {
        childs[i]->getBoundingBox(minAuxPt, maxAuxPt);
        if(maxLongTemp < maxAuxPt.X())
            maxLongTemp = maxAuxPt.X();
    }

    maxAuxPt.X() = pos.Norm() + maxLongTemp;
    return true;
}

void joint::computeWorldPos()
{
    if(dirtyFlag) update();
	((JointRender*)shading)->computeWorldPos(this);
	/*
    glPushMatrix();

    glTranslated(pos.X(),pos.Y(),pos.Z());

    glRotatef((GLfloat)orientJoint.Z(),0,0,1);
    glRotatef((GLfloat)orientJoint.Y(),0,1,0);
    glRotatef((GLfloat)orientJoint.X(),1,0,0);

    glRotatef((GLfloat)rot.Z(),0,0,1);
    glRotatef((GLfloat)rot.Y(),0,1,0);
    glRotatef((GLfloat)rot.X(),1,0,0);

    GLdouble modelview[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
    worldPosition = Point3d(modelview[12],modelview[13],modelview[14]);

    for(unsigned int i = 0; i< childs.size(); i++)
    {
        childs[i]->computeWorldPos();
    }

    glPopMatrix();
	*/
}

void joint::setJointOrientation(double ojX,double  ojY,double  ojZ)
{
    orientJoint = Point3d(ojX, ojY, ojZ);
}

void joint::getRelatives(vector<joint*>& joints)
{
    joints.push_back(this);

    for(unsigned int i = 0; i< childs.size(); i++)
        childs[i]->getRelatives(joints);
}

//JOINT
void joint::drawFunc()
{    
	
    if(dirtyFlag) update();

	((JointRender*) shading)->drawFunc(this);

	/*
    //double l = pos.Norm();

    glColor3f(1.0,1.0,1.0);

    glPushMatrix();

    //double alpha = 0;
    //if(pos.X() > 0)
    //    alpha = atan(pos.Z()/pos.X())*(360/(2*M_PI));

    //double beta = 0;
    //if(l > 0)
    //    beta = asin(pos.Y()/l)*(360/(2*M_PI));

    //glRotatef(-alpha,0,1,0);
    //glRotatef(-beta,0,0,1);

    if(father)
    {
        if(father->shading->selected)
            glColor3f((GLfloat)SELR,(GLfloat)SELG,(GLfloat)SELB);
        else
            glColor3f(NORMALR,NORMALG,NORMALB);

        drawLine(pos.X(), pos.Y(),pos.Z());
        //drawBone(l,DEFAULT_SIZE);
    }

    glTranslated(pos.X(),pos.Y(),pos.Z());

    glRotatef((GLfloat)orientJoint.Z(),0,0,1);
    glRotatef((GLfloat)orientJoint.Y(),0,1,0);
    glRotatef((GLfloat)orientJoint.X(),1,0,0);

    glRotatef((GLfloat)rot.Z(),0,0,1);
    glRotatef((GLfloat)rot.Y(),0,1,0);
    glRotatef((GLfloat)rot.X(),1,0,0);

    // Pintamos un tri-círculo
    if(shading->selected)
        glColor3f((GLfloat)SELR,(GLfloat)SELG,(GLfloat)SELB);
    else
        glColor3f(NORMALR,NORMALG,NORMALB);

    // Pintamos 3 los círculos
    drawTriCircle(12, DEFAULT_SIZE);

    // Pintamos los ejes del hueso
    drawAxisHandle(DEFAULT_SIZE*0.7);

    // Pintamos la pelota de expansion
    drawExpansionBall(shading->selected, (float)(DEFAULT_SIZE*2*expansion));

    for(unsigned int i = 0; i< childs.size(); i++)
    {
        childs[i]->drawFunc();
    }

    glPopMatrix();
	*/
}

void joint::select(bool bToogle, int id)
{
    bToogle |= ((unsigned int)id == nodeId);

    shading->selected = bToogle;
    for(unsigned int i = 0; i< childs.size(); i++)
        childs[i]->select(bToogle, id);
}

// SKELETON
//void skeleton::update()
//{

    // Calcular matrix de transformacion
//    dirtyFlag = false;
//}

skeleton::skeleton() : object()
{
    joints.clear();
    root = NULL;
    iam = SKELETON_NODE;

	shading = new SkeletonRender(this);
}

skeleton::skeleton(unsigned int nodeId) : object(nodeId)
{
    joints.clear();
    root = NULL;
    iam = SKELETON_NODE;

	shading = new SkeletonRender(this);
}

skeleton::~skeleton()
{
    delete root;
    joints.clear();
}

void skeleton::drawFunc()
{
    if(dirtyFlag) update();

    root->drawFunc();
    // Aplicamos las rotaciones y traslaciones pertinentes
}

bool skeleton::getBoundingBox(Point3d& minAuxPt,Point3d& maxAuxPt)
{
    // Calculamos la mayor caja que podría darse con la posición
    // inicial del equeleto. Esto es si todos los huesos pueden estirarse.
    maxAuxPt = Point3d(0,0,0);
    for(int i = 0; i< root->getChildCount(); i++)
    {
        root->getChild(i)->getBoundingBox(minAuxPt, maxAuxPt);
    }

    double l = maxAuxPt.X() * cos(45*2*M_PI/360);
    if(l <= 0)
        l = 1;

    minAuxPt = root->pos - Point3d(l,l,l);
    maxAuxPt = root->pos + Point3d(l,l,l);

    return true;
}

void skeleton::select(bool bToogle, int id)
{
    root->select( bToogle, id);
}

bool skeleton::update()
{
	if(shading)
		((SkeletonRender*)shading)->updateSkeleton(this);

    return true;
}

/*
vector<DefNode>& nodes = ((gridRenderer*)escena->visualizers[i])->grid->v.intPoints;

for(unsigned int n = 0; n< nodes.size(); n++)
{
    if(((DefNode)nodes[n]).boneId == (int)jt->nodeId)
    {
        if(((DefNode)nodes[n]).ratio < ratioExpansion_DEF)
        {
            float ratio2 = (((DefNode)nodes[n]).ratio/ratioExpansion_DEF);
            float dif = 1-expValue;
            float newValue =  expValue + dif*ratio2;
            nodes[n].expansion = newValue;
        }
    }
}
*/

int subdivideBone(joint* parent, joint* child,
				  //Point3d origen, Point3d fin,
				  vector< DefNode >& nodePoints,
				  vector<int>& ids,
				  float subdivisionRatio)
{
	//float subdivisionRatio = subdivisionRatio_DEF;
	Point3d origen =  parent->getWorldPosition();
	Point3d fin = child->getWorldPosition();
	int boneId = parent->nodeId;

	double longitud= (float)((fin-origen).Norm());
	double endChop = longitud*endChop_DEF;

	if(longitud == 0)
		return 0;

	Point3d dir = (fin-origen)/longitud;
	longitud = longitud - endChop;
	int numDivisions = (int)floor(longitud/subdivisionRatio);

	double newSubdivLength = longitud;
	if(numDivisions > 0)
		newSubdivLength = longitud/ double(numDivisions);

	Point3d newOrigen = origen;
	Point3d newFin = fin-dir*endChop;

	// Añadimos los nodos
	// Saltamos el primer nodo que corresponde a la raíz del joint
	for(int i = 1; i<= numDivisions; i++)
	{
		//int globalNodeId = nodePoints.size();
		ids.push_back(nodePoints.size());	
		nodePoints.push_back(DefNode(newOrigen+(dir*newSubdivLength*i),boneId));
		nodePoints.back().nodeId = scene::getNewId();
		nodePoints.back().ratio = (float)i/(float)numDivisions;

		// Expansion propagation
		float expValue = parent->expansion;
		float expValue2 = child->expansion;

		// Queda por decidir si queremos continuidad entre las expansiones o no.
		float ratio2 = nodePoints.back().ratio/ratioExpansion_DEF;
		if(ratio2 > 1) ratio2 = 1;
		if(ratio2 < 0) ratio2 = 0;

        float dif = 1-expValue;
        float newValue =  expValue + dif*ratio2;

		nodePoints.back().expansion = newValue;
	}

	// Comprobamos que el último punto sea lo que toca...
	//assert(newfin == nodePoints.back());

	if(VERBOSE)
	{
		float error = (float)(newFin - nodePoints.back().pos).Norm();
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

void addNodes(joint* jt, vector< DefNode >& nodePoints, vector<int>& idx,
			  float subdivisionRatio)
{
	// We add a node in the joint pos.
	idx.push_back(nodePoints.size());
	nodePoints.push_back(DefNode(jt->getWorldPosition(), jt->nodeId));
	nodePoints.back().nodeId = scene::getNewId();
	nodePoints.back().ratio = 0.0;
	nodePoints.back().expansion = jt->expansion;

	// We link all the child nodes to this node.
	for(int i = 0; i < jt->getChildCount(); i++)
	{
		int numDivisions;
		numDivisions = subdivideBone(jt, jt->getChild(i), 
			nodePoints, idx, subdivisionRatio);

		addNodes(jt->getChild(i), nodePoints, idx, subdivisionRatio);
	}
}

// A Partir del esqueleto define unos nodos para la segmentacion
// IN: skt
// OUT: nodePoints
int proposeNodes(vector<skeleton*>& skts, vector< DefNode >& nodePoints)
{
	// Initialization
	nodePoints.clear();

	vector< int > nodePointsIdx;
	for(unsigned int sk = 0; sk < skts.size(); sk++)
	{
		vector<int> skNodePointsIdx;
		addNodes(skts[sk]->root, nodePoints, skNodePointsIdx, skts[sk]->minSegmentLength);
		for(unsigned int skNPidx = 0; skNPidx<skNodePointsIdx.size(); skNPidx++)
			nodePointsIdx.push_back(sk);
	}

	// The nodes are linked to the bones.
	for(unsigned int j = 0; j < nodePoints.size(); j++)
	{
		joint* jt = skts[nodePointsIdx[j]]->getJoint(nodePoints[j].boneId);
		if(jt == NULL)
		{
			printf("There is a problem.\n"); fflush(0);
		}
		else
		{
			jt->nodes.push_back(&nodePoints[j]);
			int prie = 0;
		}
	}

	// Devolvemos el numero de nodos añadidos.
	return nodePoints.size();
}

 float GetMinSegmentLenght(float minLength, float cellSize)
 {
	if(minLength > cellSize*3)
		return minLength-0.01;

	else
		return cellSize;
 }

 float getMinSkeletonSize(skeleton* skt)
 {
	float minlength = -1;
	for(int i = 0; i< skt->joints.size(); i++)
	{
		if(skt->joints[i]->father)
		{
			if(minlength < 0)
				minlength = (skt->joints[i]->father->getWorldPosition()-skt->joints[i]->getWorldPosition()).Norm();
			else
			{
				float length = (skt->joints[i]->father->getWorldPosition()-skt->joints[i]->getWorldPosition()).Norm();
				if(length < minlength)
					minlength = length;
			}
		}
	}

	return minlength;
 }