#include "outliner.h"

#include "Scene.h"
#include "Modelo.h"
#include "skeleton.h"

void outliner::getSceneTree(joint* j,treeNode* root)
{
    root->sName = j->sName;
    root->nodeId = j->nodeId;
    root->type = NODETYPE_JOINT;

    for(int i = 0; i< j->getChildCount(); i++)
    {
        treeNode* childnode = new treeNode();
        getSceneTree(j->getChild(i), childnode);
        root->childs.push_back(childnode);
    }
}

void outliner::getSceneTree(skeleton* s,treeNode* root)
{
    root->sName = s->root->sName;
    root->nodeId = s->root->nodeId;
    root->type = NODETYPE_JOINT;

    for(int i = 0; i< s->root->getChildCount(); i++)
    {
        treeNode* childnode = new treeNode();
        getSceneTree(s->root->getChild(i), childnode);
        root->childs.push_back(childnode);
    }
}

void outliner::getSceneTree(Modelo* m,treeNode* root)
{
    root->sName = m->sName;
    root->nodeId = m->nodeId;
    root->type = NODETYPE_MODEL;

    if(m->modelCage)
    {
        treeNode* modelCage = new treeNode();
        modelCage->sName = m->modelCage->sName;
        modelCage->nodeId = m->modelCage->nodeId;
        modelCage->type = NODETYPE_CAGE;
        root->childs.push_back(modelCage);
    }

    if(m->dynCage)
    {
        treeNode* dynCage = new treeNode();
        dynCage->sName = m->dynCage->sName;
        dynCage->nodeId = m->dynCage->nodeId;
        dynCage->type = NODETYPE_CAGE;
        root->childs.push_back(dynCage);
    }

    for(unsigned int i = 0; i< m->stillCages.size(); i++)
    {
        treeNode* stcage = new treeNode();
        stcage->sName = m->stillCages[i]->sName;
        stcage->nodeId = m->stillCages[i]->nodeId;
        stcage->type = NODETYPE_CAGE;
        root->childs.push_back(stcage);
    }

}

void outliner::getSceneTree(treeNode* root)
{
    if(!escena) return;

    root->childs.push_back(new treeNode());
    treeNode* m = (treeNode*)root->childs.back();
    m->sName = "models";
    m->nodeId = 0;
    m->type = NODETYPE_MODELROOT;

    for(unsigned int i = 0; i< escena->models.size(); i++)
    {
        treeNode* ch = new treeNode();
        getSceneTree((Modelo*)escena->models[i], ch);
        m->childs.push_back(ch);
    }

    root->childs.push_back(new treeNode());
    treeNode* skt = (treeNode*)root->childs.back();
    skt->sName = "skeletons";
    skt->nodeId = 0;
    skt->type = NODETYPE_SKELETON;

    for(unsigned i = 0; i< escena->skeletons.size(); i++)
    {
        treeNode* ch = new treeNode();
        getSceneTree((skeleton*)escena->skeletons[i], ch);
        skt->childs.push_back(ch);
    }

}
