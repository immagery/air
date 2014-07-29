#ifndef AIR_MANIPULATOR_H
#define AIR_MANIPULATOR_H

#include <DataStructures\Node.h>
#include <DataStructures\Object.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen\Geometry>

enum manipulatorType {MANIP_NONE = 0, MANIP_MOVE, MANIP_ROT, MANIP_SCL};
enum manipulatorAxis {AXIS_X = 0, AXIS_Y, AXIS_Z, AXIS_VIEW};
enum transformMode {TM_WORLD = 0, TM_OBJECT, TM_SINGLE, TM_HIERARCHY};

using namespace Eigen;

class airFrame
{
public:
	Vector3d position;
	Quaterniond rotation;
	Vector3d scale;

	Vector3d posOffset;
	Quaterniond rotOffset;

	void getMatrix(Matrix4d& mtx);
};

class manipulator : public node
{
public:

	manipulator();

	airFrame currentframe;
	airFrame previousframe;

	void setCurrentFrame(Vector3d, Quaterniond, Vector3d);
	void setFrame(Vector3d, Quaterniond, Vector3d);

	manipulatorType type;
	manipulatorAxis axis;

	// this vatiable configures how to move, 
	//depending on the hierarchy, or just the selected object.
	transformMode mode;

	bool bModifierMode;
	bool bEnable;

	Vector2i pressMouse;

	Vector3d orig, dir, selectedPoint, projectedPoint, projectedPoint2;

	Vector3d generalPurpousePoint001, generalPurpousePoint002;

	vector<Vector3d > jointsPoints;
	vector<Vector3d > jointsColors;

	// For VIEW mode interaction
	Vector3d cameraPos;

	float size;
	virtual void drawFunc();
	virtual void drawFuncNames();
	virtual void drawNamesWithProjectingPlane();

	void startManipulator(Vector3d& rayOrigin, Vector3d& rayDir, bool withOffset = false);
	void setManipulator(Vector3d& rayOrigin, Vector3d& rayDir, bool withOffset = false);
	void moveManipulator(Vector3d& rayOrigin, Vector3d& rayDir);

	void applyTransformation(object* obj, manipulatorType type, transformMode mode = TM_WORLD);
};


#endif