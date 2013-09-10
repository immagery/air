#ifndef OUTLINER_H
#define OUTLINER_H

#include "vector"
#include "string"

class scene;
class Modelo;
class skeleton;
class joint;

enum outlinesNodeTypes {NODETYPE_MODEL = 0, NODETYPE_CAGE = 0, NODETYPE_SKELETON, NODETYPE_JOINT,
                        NODETYPE_CURVE,     NODETYPE_HANDLE,   NODETYPE_CAM,
                        NODETYPE_MODELROOT, NODETYPE_SKELETONROOT};

using namespace std;

class treeNode
{
public:
    treeNode(){}

    string sName;
    int nodeId;
    outlinesNodeTypes type;
    vector< treeNode* > childs;
};

class outliner
{
public:

    outliner(){}

    outliner(scene* scena){ escena = scena;}

    void getSceneTree(treeNode* root);

private:
    void getSceneTree(joint* j,treeNode* root);
    void getSceneTree(skeleton* s,treeNode* root);
    void getSceneTree(Modelo* m  ,treeNode* root);

    scene* escena;
};

#endif // OUTLINER_H
