#ifndef BULGE_DEFORMER_H
#define BULGE_DEFORMER_H

// Std lib
#include <vector>

// Air DataStructures
#include <DataStructures\node.h>

// Creacion de curvas spline 2D (+BOOST)
#include <Computation/spline.hpp>

// Eigen Data Structures
#include <Eigen/Dense>

using namespace std;
using namespace magnet::math;


class wireDeformer : public node
{
public:
	wireDeformer();

	// Deformation curves
	Spline W; // New Curve
	Spline R; // Reference

	float r; // radius
	float s; // scale

	float lenght;
	float high;

	// Axis: to work with +x,+y 2D plane (z as normal vector)
	Quaterniond orient;
	Vector3d pos;

	Vector3d mDir,mN;
	
	Vector3d D,origin;

	float implicitFunction(float value);

	// Computes the deformation.
	void getDisplacement(Vector3d pt, float u, float w, Vector3d& displ);

	// Initzialization of axis for compute deformations
	void init(Vector3d dir, Vector3d n, vector<Vector3d>& curvePoints, vector<Vector3d>& curveCoords, float maxPressureChild);
};

class BulgeGroup
{
public:
	BulgeGroup();

	// Which deformers play with this group
	int deformerId;
	int childDeformerId;

	bool direction;

	// The vertex influeced in this Bulge Group
	vector<int> vertexGroup;

	// Weights: pairs vertex, deformer
	vector<float> w;

	// Parametrization
	vector<float> u;

	// Control points for each curve
	vector<Vector3d> refPoints;
	vector<Vector3d> refCoords;
	vector<int> triangleSequence;

	vector<Vector3d> defPoints;

	// WireDeformationCurve
	wireDeformer* defCurve;

	// Pressure measurement
	vector<Vector2f> pressurePoints;
	vector<float> pressure;
	Vector2f minI, maxI;

};

class Geometry;
class binding;
class AirRig;

class BulgeDeformer : public node
{
public:

	// Funciones
	BulgeDeformer();
	void applyDeformation(Geometry* m, Geometry* origM,binding* b, AirRig* rig);

	// Each joint has at least two groups-> father->joint, joint->child
	vector<BulgeGroup*> groups;

	// but it could be used with 1 termination (something like an elephant limb walking)
	// and also with more than 1 branch, for hands and complex joints.

	bool enabled;
};

#endif // BULGE_DEFORMER_H