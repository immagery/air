#ifndef DEFGROUP_H
#define DEFGROUP_H

#include <DataStructures/DefNode.h>
#include <DataStructures/skeleton.h>

#define endChop_DEF 0.01
#define ratioExpansion_DEF 1.0

#define default_SMOOTHING_PASES 3
#define default_SMOOTHING_RATIO 0.25

#define default_SUBDIVISION_RATIO 0.25
#define default_EXPANSION 1

#define default_INI_TWIST 0.0
#define default_END_TWIST 1.0

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
	bool dirtySmooth;

	DefGroupType type; 

	// For computational pourposes.
	vector<DefGroup*> dependentGroups;
	vector<DefGroup*> relatedGroups;

	map<unsigned int, DefGroup*>* references; 

	defGroupSerialization* serializedData;

	bool saveToFile(FILE* fout);
	bool loadFromFile(ifstream& in, airRigSerialization* sData);

	virtual void setRotation(double rx, double ry, double rz, bool radians);
	
	virtual void addTranslation(double tx, double ty, double tz);
	virtual void setTranslation(double tx, double ty, double tz, bool local);
	virtual void addRotation(double rx, double ry, double rz);
	virtual void addRotation(Eigen::Quaternion<double> q, bool local = true);
	virtual void setRotation(Eigen::Quaternion<double> q, bool local = true);

	Quaterniond getRotation(bool local = true);
	Vector3d getTranslation(bool local = true);

	Vector3d getRestTranslation(bool local = true);
	Quaterniond getRestRotation(bool local = true);

	bool dirtyByTransformation(bool alsoFather, bool hierarchically = true);
	bool dirtyBySegmentation();
	bool dirtyBySmoothing();

	void saveRestPos(DefGroup* dg, DefGroup* father = NULL);
	void computeRestPos(DefGroup* dg, DefGroup* father = NULL);
	
	void computeWorldPos(DefGroup* dg, DefGroup* father = NULL);
	void computeWorldPosNonRoll(DefGroup* dg, DefGroup* father = NULL);

	void restorePoses(DefGroup* dg, DefGroup* father = NULL);

	virtual bool propagateDirtyness();

    virtual bool update();

	virtual bool select(bool bToogle, unsigned int id);
	virtual bool selectRec(bool bToogle);

	//void computeWorldPosRec(DefGroup* dg, DefGroup* fatherDg);

};


#endif // DEFGROUP_H