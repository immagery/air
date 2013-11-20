#include "AirOutliner.h"

#include <dataStructures/Scene.h>
#include <dataStructures/Modelo.h>
#include <dataStructures/skeleton.h>
#include <dataStructures/AirRig.h>

void AirOutliner::getSceneTree(DefGroup* group,treeNode* root)
{
    root->sName = group->sName;
    root->nodeId = group->nodeId;
	root->type = NODETYPE_RIG_DEFGROUP;

	for(int i = 0; i< group->relatedGroups.size(); i++)
    {
        treeNode* childnode = new treeNode();
        getSceneTree(group->relatedGroups[i], childnode);
        root->childs.push_back(childnode);
    }
}

void AirOutliner::getSceneTree(treeNode* root)
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
        outliner::getSceneTree((Modelo*)escena->models[i], ch);
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
        outliner::getSceneTree((skeleton*)escena->skeletons[i], ch);
        skt->childs.push_back(ch);
    }

	int id = 0;
    root->childs.push_back(new treeNode());
    treeNode* rig = (treeNode*)root->childs.back();
    rig->sName = "rig";
    rig->nodeId = id; id++;
    rig->type = NODETYPE_RIGROOT;

    rig->childs.push_back(new treeNode());
    treeNode* defGraph = (treeNode*)rig->childs.back();
    defGraph->sName = "Def_Graph";
    defGraph->nodeId = id; id++;
	defGraph->type = NODETYPE_RIG_DEFGRAPH;

	AirRig* rigStr = (AirRig*)escena->rig;

	for(int i = 0; i< rigStr->defRig.roots.size(); i++)
	{
        treeNode* ch = new treeNode();
        getSceneTree(rigStr->defRig.roots[i], ch);
        defGraph->childs.push_back(ch);
	}

	rig->childs.push_back(new treeNode());
    treeNode* controlGraph = (treeNode*)rig->childs.back();
    controlGraph->sName = "Control_Graph";
    controlGraph->nodeId = id; id++;
	controlGraph->type = NODETYPE_RIG_CONTROLGRP;

}

