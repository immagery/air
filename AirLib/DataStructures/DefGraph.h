#ifndef DEFGRAPH_H
#define DEFGRAPH_H

#include <DataStructures/Node.h>
#include <DataStructures/DefGroup.h>
#include <DataStructures/AirConstraint.h>

class DefGraph : public node
{
public:
	DefGraph();

	vector<DefGroup*> roots;

	vector<joint*> joints;
	vector<DefGroup*> defGroups;

	vector<DefNode*> deformers;
	vector<Constraint* > relations;

	map<unsigned int, DefGroup*> defGroupsRef;
	map<unsigned int, DefNode*> defNodesRef;

	int smoothingPasses;
	float smoothPropagationRatio;

	bool saveToFile(FILE* fout);
	bool loadFromFile(ifstream& in, airRigSerialization* sData);

};

#endif // DEFGRAPH_H
