#include "DefGraph.h"

#include <DataStructures/Scene.h>

//////////////////
//   DEFGRAPH   //
//////////////////

DefGraph::DefGraph()
{
	roots.clear();

	joints.clear();
	defGroups.clear();

	deformers.clear();
	relations.clear();

	defGroupsRef.clear();

	smoothingPasses = 3;
	smoothPropagationRatio = 0.25;

	iam = DEFGRAPH_NODE;
}

bool DefGraph::saveToFile(FILE* fout)
{
	if(!fout)
	{
		printf("There is no file to print!\n [AIR_RIG]");
		return false;
	}

	node::saveToFile(fout);

	fprintf(fout, "%d %d\n", defGroups.size(), relations.size()); fflush(fout);
	for(int groupIdx = 0; groupIdx < defGroups.size(); groupIdx++)
	{
		defGroups[groupIdx]->saveToFile(fout);
	}

	for(int relationIdx = 0; relationIdx < relations.size(); relationIdx++)
	{
		relations[relationIdx]->saveToFile(fout);
	}

	/* 
		Important to ensure the data consistency
		vector<DefGroup*> roots; -> building the tree
		vector<joint*> joints; -> binding the skeletons
		vector<DefNode*> deformers; -> looking to the groups
		map<unsigned int, DefGroup*> defGroupsRef;
		map<unsigned int, DefNode*> defNodesRef;
	*/

	return true;
}

// Loads data from this file, it is important to bind
// this data with the model and skeleton after this function.
bool DefGraph::loadFromFile(ifstream& in, airRigSerialization* sData)
{
	node::loadFromFile(in);

	int relationsSize;
	int defGroupsSize;

	string line;
	vector<string> elems;

	getline (in , line);
    split(line, ' ', elems);
	defGroupsSize = atoi(elems[0].c_str());
	relationsSize = atoi(elems[1].c_str());

	//fscanf(in, "%d %d", defGroupsSize, relationsSize); 

	defGroups.resize(defGroupsSize);
	for(int groupIdx = 0; groupIdx < defGroups.size(); groupIdx++)
	{
		defGroups[groupIdx] = new DefGroup(scene::getNewId(T_DEFGROUP));
		if(!defGroups[groupIdx]->loadFromFile(in, sData)) return false;
	}

	relations.resize(relationsSize);
	for(int relationIdx = 0; relationIdx < relations.size(); relationIdx++)
	{
		relations[relationIdx] = new Constraint();
		if(!relations[relationIdx]->loadFromFile(in)) return false;
	}

	/* 
		Important to ensure the data consistency
		vector<DefGroup*> roots; -> building the tree
		vector<joint*> joints; -> binding the skeletons
		vector<DefNode*> deformers; -> looking to the groups
		map<unsigned int, DefGroup*> defGroupsRef;
		map<unsigned int, DefNode*> defNodesRef;
	*/

	return true;
}