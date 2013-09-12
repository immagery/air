#ifndef SCENE_H
#define SCENE_H

#include "Deformer.h"
#include "Cage.h"
#include "skeleton.h"
#include "Modelo.h"

// reservamos unos cuantos números para elementos especiales
#define FIRST_NODE_ID 100

enum procState{CREATED = 0, RIGG, ANIMATION, INTERACTIVE};

class sceneUpdatingFlags
{
public:
	bool updateDefNodes;
	bool updateSkinningFlag;
	vector<int> defNodesToUpdate;
	bool all;

	sceneUpdatingFlags()
	{
		updateDefNodes = false;
		updateSkinningFlag = false;
		all = false;
		defNodesToUpdate.clear();
	}
};

class defInfo
{
	public:

	int id;
	Point3d position;
	float smooth;
	float expansion;
	int relation[2];

	defInfo()
	{
		id = -1;
		smooth = 1;
		expansion = 1;
		position = Point3d(0,0,0);
		relation[0] = -1;
		relation[1] = -1;
	}

	defInfo(const defInfo& inD)
	{	
		smooth = inD.smooth;
		expansion = inD.expansion;
		id = inD.id;
		position = inD.position;
		relation[0] = inD.relation[0];
		relation[1] = inD.relation[1];
	}
};

class inData
{
	public:
	bool firstTime;

	float globalScale;
	float globalSmooth;

	procState mode;

	vector<defInfo> deformers;

	inData()
	{
		deformers.clear();
		firstTime = true;
		globalScale = -1;
		globalSmooth = -1;
	}
};

// Total Scene
class scene
{
    public:
    scene()
    {
        scene::sceneIds = FIRST_NODE_ID;
		modelLoaded = false;
		skeletonLoaded = false;
		embeddingLoaded = false;
		weightsUpdated = false;
		initialized = false;
		evaluate = false;
		gridLoadedFromDisc = false;
		state = CREATED;
		weightsThreshold = -10;

		iVisMode = 0;
		desiredVertex = 0;
    }

    ~scene()
    {

		printf("Destruccion de la escena\n"); fflush(0);

        for(unsigned int i = 0; i< models.size(); i++)
            delete models[i];

        for(unsigned int i = 0; i< skeletons.size(); i++)
            delete skeletons[i];

        for(unsigned int i = 0; i< deformers.size(); i++)
            delete deformers[i];

        for(unsigned int i = 0; i< shaders.size(); i++)
            delete shaders[i];

        models.clear();
        skeletons.clear();
        deformers.clear();
        shaders.clear();
    }

    static unsigned int getNewId(){scene::sceneIds++; return scene::sceneIds-1;}


	void setSceneScale( float sceneScale);
	void setGlobalSmoothness(float globalSmooth);

    void selectElements(vector<unsigned int > lst)
    {
        for(unsigned int j = 0; j < lst.size(); j++)
        {
            for(unsigned int i = 0; i< models.size(); i++)
                ((Modelo*)(models[i]))->select(true, lst[j]);

            for(unsigned int i = 0; i< skeletons.size(); i++)
                ((skeleton*)(skeletons[i]))->select(false, lst[j]);
        }
    }

    void removeSelection()
    {
        for(unsigned int i = 0; i< models.size(); i++)
            ((Modelo*)(models[i]))->select(false, -1);

        for(unsigned int i = 0; i< skeletons.size(); i++)
            ((skeleton*)(skeletons[i]))->select(false, -1);
    }

	//Models
    vector< object*   > models;
    vector< deformer* > deformers;

	// Render
    vector< shadingNode*   > visualizers;
    vector< shadingNode* > shaders;

	// Scene management
    static unsigned int sceneIds; // ids que va repartiendo
	map<int, int> defIds; // tranlation for change IDs
	shadingNode* currentSelection;

	// Skeleton
	map<int, joint*> deformerRef; // A reference to all the joints from id.
	vector< skeleton* > skeletons;

	procState state;

	// Scene state flags
	bool modelLoaded;
	bool skeletonLoaded;
	bool embeddingLoaded;
	bool weightsUpdated;

	double weightsThreshold;

	bool gridLoadedFromDisc;
	bool initialized;
	bool evaluate;

	//inData inConnexionsTemp;

	void ( *fPrintText)(char*);
	void ( *fPrintProcessStatus)(int);

	int iVisMode;
	int desiredVertex;

};

class camera : object
{
    public:
        vcg::Point3d target;
};

#endif // SCENE_H
