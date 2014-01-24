#ifndef AIR_MANIPULATOR_H
#define AIR_MANIPULATOR_H

#include <DataStructures\Node.h>
#include <DataStructures\Object.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen\Geometry>

enum manipulatorType {MANIP_NONE = 0, MANIP_MOVE, MANIP_ROT, MANIP_SCL};
enum manipulatorAxis {AXIS_X = 0, AXIS_Y, AXIS_Z, AXIS_VIEW};
enum transformMode {TM_WORLD = 0, TM_OBJECT};

using namespace Eigen;

class airFrame
{
public:
	Vector3d position;
	Quaterniond rotation;
	Vector3d scale;

	void getMatrix(Matrix4d& mtx);
};

class manipulator : public node
{
public:

	manipulator()
	{
		bModifierMode = false;
		size = 1.0;
		type = MANIP_NONE;
		axis = AXIS_X;
		bEnable = false;

		projectedPoint = Vector3d(0,0,0);
	}

	airFrame currentframe;
	airFrame previousframe;

	void setCurrentFrame(Vector3d, Quaterniond, Vector3d);
	void setFrame(Vector3d, Quaterniond, Vector3d);

	manipulatorType type;
	manipulatorAxis axis;

	bool bModifierMode;
	bool bEnable;

	Vector2i pressMouse;

	Vector3d orig, dir, selectedPoint, projectedPoint;

	float size;
	virtual void drawFunc();
	virtual void drawFuncNames();
	virtual void drawNamesWithProjectingPlane();

	void moveManipulator(Vector3d& rayOrigin, Vector3d& rayDir);

	void applyTransformation(object* obj, transformMode mode = TM_WORLD);
};


#endif