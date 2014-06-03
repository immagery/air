#ifndef DEFCONSTRAINT_H
#define DEFCONSTRAINT_H

#include <iostream>
#include <fstream>

#include <DataStructures/DefGroup.h>

class constraintSerialization
{
public:
	int parentId;
	int childId;
};

class Constraint : public node
{
public:
	Constraint()
	{
		weight = 1.0;
	}

	DefGroup* parent;
	DefGroup* child;
	int weight;

	constraintSerialization* sConstraint;

	// Serialization
	virtual bool saveToFile(FILE* fout);

	// How can a I load the references.
	virtual bool loadFromFile(ifstream& in);

};

class PointConstraint : public Constraint
{
public:
	Vector3d offset;
	Vector3d value;

	PointConstraint() : Constraint()
	{
		offset = Vector3d(0,0,0);
		value = Vector3d(0,0,0);
	}

	// Serialization
	virtual bool saveToFile(FILE* fout)
	{ 
		fout = fout; // for delete warning
		return true;
	}
	virtual bool loadFromFile(ifstream& in)
	{ 
		return true;
	}

};

class OrientConstraint : public Constraint
{
public:
	Vector3d offset;
	Vector3d value;

	OrientConstraint() : Constraint()
	{
		offset = Vector3d(0,0,0);
		value = Vector3d(0,0,0);
	}

	// Serialization
	virtual bool saveToFile(FILE* fout)	
	{ 
		fout = fout; // for delete warning
		return true;
	}

	virtual bool loadFromFile(ifstream& in)	
	{ 
		return true;
	}
};

class ScaleConstraint : public Constraint
{
public:
	Vector3d offset;
	Vector3d value;

	ScaleConstraint() : Constraint()
	{
		offset = Vector3d(0,0,0);
		value = Vector3d(0,0,0);
	}

	// Serialization
	virtual bool saveToFile(FILE* fout)	
	{ 
		fout = fout; // for delete warning
		return true;
	}
	virtual bool loadFromFile(ifstream& in)	
	{ 
		return true;
	}
};

class ParentConstraint : public Constraint
{
public:
	OrientConstraint or;
	PointConstraint pos;

	ParentConstraint() : Constraint()
	{
	}

	// Serialization
	virtual bool saveToFile(FILE* fout)	
	{ 
		fout = fout; // for delete warning
		return true;
	}
	virtual bool loadFromFile(ifstream& in)	
	{ 
		return true;
	}
};

class HierarchyConstraint : public ParentConstraint
{
public:
	ScaleConstraint slc;

	HierarchyConstraint() : ParentConstraint()
	{
	}

	// Serialization
	virtual bool saveToFile(FILE* fout)	
	{ 
		fout = fout; // for delete warning
		return true;
	}
	virtual bool loadFromFile(ifstream& in)	
	{ 
		return true;
	}
};

#endif // DEFCONSTRAINT_H