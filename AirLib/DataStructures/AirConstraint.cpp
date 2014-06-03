#include "AirConstraint.h"

// Serialization
bool Constraint::saveToFile(FILE* fout)
{ 
	node::saveToFile(fout);

	fprintf(fout, "%d %d %f\n", parent->nodeId, child->nodeId, weight);
	fflush(fout);
	return true;
}


// How can a I load the references.
bool Constraint::loadFromFile(ifstream& in)
{
	node::loadFromFile(in);

	// We save the id for restore later the references.
	float in_weight = 0;

	sConstraint = new constraintSerialization();

	string line;
	vector<string> elems;

	getline (in , line);
    split(line, ' ', elems);

	sConstraint->parentId = atoi(elems[0].c_str());
	sConstraint->childId = atoi(elems[1].c_str());
	weight = atof(elems[2].c_str());

	// need to update
	dirtyFlag = true;

	return true;
}