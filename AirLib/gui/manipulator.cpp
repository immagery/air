#include "manipulator.h"
#include <utils\utilGL.h>
#include <utils\util.h>

using namespace Eigen;

void airFrame::getMatrix(Matrix4d& mtx)
{
	Eigen::Matrix3d rotationMatrix = rotation.toRotationMatrix();

	Eigen::Matrix4d transformMatrix; 
	mtx << rotationMatrix(0,0) , rotationMatrix(0,1) , rotationMatrix(0,2), position[0],
					    rotationMatrix(1,0) , rotationMatrix(1,1) , rotationMatrix(1,2), position[1],
					    rotationMatrix(2,0) , rotationMatrix(2,1) , rotationMatrix(2,2), position[2],
						0.0,				0.0,					0.0,				1.0;

	mtx.transposeInPlace();
	/*
	transformMatrix.transposeInPlace();
	
	GLdouble multiplyingMatrix[16] = {transformMatrix(0,0), transformMatrix(0,1), transformMatrix(0,2), transformMatrix(0,3),
										transformMatrix(1,0), transformMatrix(1,1), transformMatrix(1,2), transformMatrix(1,3),
										transformMatrix(2,0), transformMatrix(2,1), transformMatrix(2,2), transformMatrix(2,3),
										transformMatrix(3,0), transformMatrix(3,1), transformMatrix(3,2), transformMatrix(3,3)
									};

	for(int i = 0; i< 16; i++)
	{
		multiplyingMatrix[i] = multiplyingMatrix2[i];
	}

	//glMultMatrixd(multiplyingMatrix);
	*/
}


void manipulator::drawNamesWithProjectingPlane()
{
	/*
	if(!bEnable) return;

	glDisable(GL_DEPTH_TEST);

	glPushMatrix();

	Matrix4d transformMatrix;
	currentframe.getMatrix(transformMatrix);

	GLdouble multiplyingMatrix[16] = {transformMatrix(0,0), transformMatrix(0,1), transformMatrix(0,2), transformMatrix(0,3),
									transformMatrix(1,0), transformMatrix(1,1), transformMatrix(1,2), transformMatrix(1,3),
									transformMatrix(2,0), transformMatrix(2,1), transformMatrix(2,2), transformMatrix(2,3),
									transformMatrix(3,0), transformMatrix(3,1), transformMatrix(3,2), transformMatrix(3,3)
								};

	glMultMatrixd(multiplyingMatrix);

	if(type == MANIP_NONE)
	{
		// no manipulator visible
	}
	else if(type == MANIP_MOVE)
	{
		// Build the plane with the points saved,
		// aligned with the view and using the axis and the
		// new projection of the mouse.

		glDisable(GL_LIGHTING);

		glLineWidth(size*2);
		glPushName(0.0);
		glPointSize(size*size);
		glBegin(GL_POINT);
		glColor3f(1.0,1.0,1.0);
		glVertex3d(0,0,0);
		glEnd();
		glPopName();

		glPushName(1.0);
		glBegin(GL_LINES);
		glColor3f(1.0,0,0);
		glVertex3d(0,0,0);
		glVertex3d(size,0,0);
		glEnd();
		glPopName();

		glPushName(2.0);
		glBegin(GL_LINES);
		glColor3f(0,1.0,0);
		glVertex3d(0,0,0);
		glVertex3d(0,size,0);
		glEnd();
		glPopName();

		glPushName(3.0);
		glBegin(GL_LINES);
		glColor3f(0,0,1.0);
		glVertex3d(0,0,0);
		glVertex3d(0,0,size);
		glEnd();
		glPopName();

		glLineWidth(1.0);

		glEnable(GL_LIGHTING);
	}
	else if(type == MANIP_ROT)
	{

		// Build the plane with the points saved,
		// use the plane that contains the circle... 
		// just the plane of the axis

		drawTriCircle(20, size);
	}
	else if(type == MANIP_SCL)
	{
		drawAxisHandleWithExtremes(size);
	}

	glPopMatrix();

	glEnable(GL_DEPTH_TEST);
	*/
}

void manipulator::applyTransformation(object* obj, transformMode mode)
{
	obj->qrot = currentframe.rotation;
	obj->pos = currentframe.position;

	if(obj->shading)
		obj->shading->dirtyFlag = true;

	obj->dirtyFlag = true;

	//obj->update();

	//obj->scl = currentframe.scale;
}

void manipulator::moveManipulator(Vector3d& rayOrigin, Vector3d& rayDir)
{
	if(type == MANIP_MOVE)
	{
		Vector3d u,v;
		Vector3d v1,v2;

		// Get directions
		if(axis == AXIS_X)
		{
			u = Vector3d(1,0,0);
			v1 = Vector3d(0,1,0);
			v2 = Vector3d(0,0,1);
		}
		else if(axis == AXIS_Y)
		{
			u = Vector3d(0,1,0);
			v1 = Vector3d(1,0,0);
			v2 = Vector3d(0,0,1);
		}
		else if(axis == AXIS_Z)
		{
			u = Vector3d(0,0,1);
			v1 = Vector3d(0,1,0);
			v2 = Vector3d(1,0,0);
		}
		
		u = currentframe.rotation._transformVector(u);
		v1 = currentframe.rotation._transformVector(v1);
		v2 = currentframe.rotation._transformVector(v2);

		if(fabs(v1.dot(rayDir)) < fabs(v2.dot(rayDir)))
			v = v1;
		else
			v = v2;

		Ray R;
		R.P0 = rayOrigin;
		R.P1 = rayOrigin+rayDir;

		Vector3d I; 
		int intersecFlag = intersect3D_RayPlane(R, selectedPoint, u, v, I);

		if(intersecFlag != 1) 
			return;

		Vector3d applyIncrement = I-selectedPoint;
		applyIncrement = applyIncrement.dot(u.normalized())*u;
		currentframe.position = previousframe.position + applyIncrement;

		projectedPoint = I;
	}
	else if(type == MANIP_ROT)
	{
		Vector3d u,v;

		// Get directions
		if(axis == AXIS_X)
		{
			u = Vector3d(0,1,0);
			v = Vector3d(0,0,1);
		}
		else if(axis == AXIS_Y)
		{
			u = Vector3d(0,0,1);
			v = Vector3d(1,0,0);
		}
		else if(axis == AXIS_Z)
		{
			u = Vector3d(1,0,0);
			v = Vector3d(0,1,0);
		}
		
		u = previousframe.rotation.inverse()._transformVector(u);
		v = previousframe.rotation.inverse()._transformVector(v);

		Ray R;
		R.P0 = rayOrigin;
		R.P1 = rayOrigin+rayDir;

		Vector3d I; 
		int intersecFlag = intersect3D_RayPlane(R, selectedPoint, u, v, I);

		if(intersecFlag != 1) 
			return;

		Vector3d v1, v2;
		v1 = (I-previousframe.position).normalized();
		v2 = (selectedPoint-previousframe.position).normalized();

		Quaterniond appliedRot;
		appliedRot.setFromTwoVectors(v2, v1);

		currentframe.rotation = previousframe.rotation*appliedRot.normalized();

		//Vector3d applyIncrement = I-selectedPoint;
		//applyIncrement = applyIncrement.dot(u.normalized())*u;
		//currentframe.position = previousframe.position + applyIncrement;

		projectedPoint = I;
	}
	else if(type == MANIP_SCL)
	{

	}
}

void manipulator::drawFuncNames()
{
	if(!bEnable) return;

	glDisable(GL_DEPTH_TEST);

	glPushMatrix();

	Matrix4d transformMatrix;
	currentframe.getMatrix(transformMatrix);

	GLdouble multiplyingMatrix[16] = {transformMatrix(0,0), transformMatrix(0,1), transformMatrix(0,2), transformMatrix(0,3),
									transformMatrix(1,0), transformMatrix(1,1), transformMatrix(1,2), transformMatrix(1,3),
									transformMatrix(2,0), transformMatrix(2,1), transformMatrix(2,2), transformMatrix(2,3),
									transformMatrix(3,0), transformMatrix(3,1), transformMatrix(3,2), transformMatrix(3,3)
								};

	glMultMatrixd(multiplyingMatrix);

	if(type == MANIP_NONE)
	{
		// no manipulator visible
	}
	else if(type == MANIP_MOVE)
	{
		glDisable(GL_LIGHTING);

		glLineWidth(size*2);
		glPushName(0.0);
		glPointSize(size*size);
		glBegin(GL_POINT);
		glColor3f(1.0,1.0,1.0);
		glVertex3d(0,0,0);
		glEnd();
		glPopName();

		glPushName(1.0);
		glBegin(GL_LINES);
		glColor3f(1.0,0,0);
		glVertex3d(0,0,0);
		glVertex3d(size,0,0);
		glEnd();
		glPopName();

		glPushName(2.0);
		glBegin(GL_LINES);
		glColor3f(0,1.0,0);
		glVertex3d(0,0,0);
		glVertex3d(0,size,0);
		glEnd();
		glPopName();

		glPushName(3.0);
		glBegin(GL_LINES);
		glColor3f(0,0,1.0);
		glVertex3d(0,0,0);
		glVertex3d(0,0,size);
		glEnd();
		glPopName();

		glLineWidth(1.0);

		glEnable(GL_LIGHTING);
	}
	else if(type == MANIP_ROT)
	{
		glDisable(GL_LIGHTING);
		glPushMatrix();
		//glColor3f(1.0,0,0);

		glPushName(1.0);
		drawCircle(20, size);
		glPopName();

		glRotatef(90, 0,1,0); // rotar en y
		//glColor3f(0,1.0,0);
		glPushName(3.0);
		drawCircle(20,size);
		glPopName();
		glPopMatrix();

		glPushMatrix();
		//glColor3f(0,0,1.0); // rotar en x
		glRotatef(90, 0,0,1);
		glPushName(2.0);
		drawCircle(20, size);
		glPopName();
		glPopMatrix();
		glEnable(GL_LIGHTING);
	}
	else if(type == MANIP_SCL)
	{
		drawAxisHandleWithExtremes(size);
	}

	glPopMatrix();

	glEnable(GL_DEPTH_TEST);
}

void manipulator::setCurrentFrame(Vector3d pos_, Quaterniond rot_, Vector3d scl_)
{
	currentframe.position = pos_;
	currentframe.rotation = rot_;
	currentframe.scale = scl_;
}

void manipulator::setFrame(Vector3d pos_, Quaterniond rot_, Vector3d scl_)
{
	previousframe.position = pos_;
	previousframe.rotation = rot_;
	previousframe.scale = scl_;

	currentframe.position = pos_;
	currentframe.rotation = rot_;
	currentframe.scale = scl_;
}

void manipulator::drawFunc()
{
	glDisable(GL_DEPTH_TEST);

	if(!bEnable) return;

	if(bModifierMode)
	{
		/// pintaríamos la forma en gris... dependediento del tipo de modificacion. 

		glPushMatrix();

		Matrix4d transformMatrix;
		previousframe.getMatrix(transformMatrix);
		GLdouble multiplyingMatrix[16] = {transformMatrix(0,0), transformMatrix(0,1), transformMatrix(0,2), transformMatrix(0,3),
										  transformMatrix(1,0), transformMatrix(1,1), transformMatrix(1,2), transformMatrix(1,3),
										  transformMatrix(2,0), transformMatrix(2,1), transformMatrix(2,2), transformMatrix(2,3),
										  transformMatrix(3,0), transformMatrix(3,1), transformMatrix(3,2), transformMatrix(3,3)
									};

		glMultMatrixd(multiplyingMatrix);

		glPointSize(size*1.5);
		glBegin(GL_POINTS);
		glColor3f(0.5,0.5,0.5);
		glVertex3d(0,0,0);
		glEnd();

		glPopMatrix();

		if(type == MANIP_MOVE)
		{
			glPointSize(size*1.5);
			glBegin(GL_LINE);
			glColor3f(0.5,0.5,0.5);
			glVertex3d(previousframe.position.x(),previousframe.position.y(),previousframe.position.z());
			glVertex3d(selectedPoint.x(),selectedPoint.y(),selectedPoint.z());
			glEnd();
		}
		else if(type == MANIP_ROT)
		{
			Vector3d v1, v2;

			v1 = (selectedPoint - previousframe.position).normalized()*size*5;
			v2 = (projectedPoint - previousframe.position).normalized()*size*5;

			glPointSize(size*1.5);
			glBegin(GL_LINES);
			glColor3f(0.5,0.5,0.5);
			glVertex3d(previousframe.position.x(),previousframe.position.y(),previousframe.position.z());
			glVertex3d(previousframe.position.x()+v1.x(),previousframe.position.y()+v1.y(),previousframe.position.z()+v1.z());
			glVertex3d(previousframe.position.x(),previousframe.position.y(),previousframe.position.z());
			glVertex3d(previousframe.position.x()+v2.x(),previousframe.position.y()+v2.y(),previousframe.position.z()+v2.z());
			glEnd();
		}

	}

	glPushMatrix();

	Matrix4d transformMatrix;
	currentframe.getMatrix(transformMatrix);
	GLdouble multiplyingMatrix[16] = {transformMatrix(0,0), transformMatrix(0,1), transformMatrix(0,2), transformMatrix(0,3),
									  transformMatrix(1,0), transformMatrix(1,1), transformMatrix(1,2), transformMatrix(1,3),
									  transformMatrix(2,0), transformMatrix(2,1), transformMatrix(2,2), transformMatrix(2,3),
									  transformMatrix(3,0), transformMatrix(3,1), transformMatrix(3,2), transformMatrix(3,3)
								};

	glMultMatrixd(multiplyingMatrix);

	if(type == MANIP_NONE)
	{
		// no manipulator visible
	}
	else if(type == MANIP_MOVE)
	{
		glDisable(GL_LIGHTING);

		/*glPointSize(5);
		glBegin(GL_POINT);
		glColor3f(1.0,1.0,1.0);
		glVertex3d(0,0,0);
		glEnd();
		*/
		glLineWidth(size);
		
		if(axis== AXIS_X)
			glLineWidth(size*1.2);
		else
			glLineWidth(size*0.8);

		glBegin(GL_LINES);
		glColor3f(1.0,0,0);
		glVertex3d(0,0,0);
		glVertex3d(size,0,0);
		glEnd();

		if(axis== AXIS_Y)
			glLineWidth(size*1.2);
		else
			glLineWidth(size*0.8);

		glBegin(GL_LINES);
		glColor3f(0,1.0,0);
		glVertex3d(0,0,0);
		glVertex3d(0,size,0);
		glEnd();

		if(axis== AXIS_Z)
			glLineWidth(size*1.2);
		else
			glLineWidth(size*0.8);

		glBegin(GL_LINES);
		glColor3f(0,0,1.0);
		glVertex3d(0,0,0);
		glVertex3d(0,0,size);
		glEnd();

		if(axis== AXIS_VIEW)
			glLineWidth(size*1.2);
		else
			glLineWidth(size*0.8);

		glPointSize(size*size);
		glBegin(GL_POINTS);
		glColor3f(1,1,0);
		glVertex3d(0,0,0);
		glEnd();

		glLineWidth(1.0);

		glPointSize(size*size);
		glBegin(GL_POINTS);
		glColor3f(1,1,1);
		glVertex3d(selectedPoint.x(),selectedPoint.y(),selectedPoint.z());
		glColor3f(0,1,0);
		glVertex3d(projectedPoint.x(),projectedPoint.y(),projectedPoint.z());
		glEnd();

		glEnable(GL_LIGHTING);
	}
	else if(type == MANIP_ROT)
	{

		if(axis== AXIS_X)
			glLineWidth(size*1.2);
		else
			glLineWidth(size*0.8);

		glDisable(GL_LIGHTING);
		glPushMatrix();
		glColor3f(1.0,0,0);
		drawCircle(20, size);

		if(axis== AXIS_Z)
			glLineWidth(size*1.2);
		else
			glLineWidth(size*0.8);

		glRotatef(90, 0,1,0); // rotar en y
		glColor3f(0,0.0,1.0);
		drawCircle(20, size);
		glPopMatrix();

		if(axis== AXIS_Y)
			glLineWidth(size*1.2);
		else
			glLineWidth(size*0.8);

		glPushMatrix();
		glColor3f(0,1.0,0.0); // rotar en x
		glRotatef(90, 0,0,1);
		drawCircle(20, size);
		glPopMatrix();
		glEnable(GL_LIGHTING);
	}
	else if(type == MANIP_SCL)
	{
		drawAxisHandleWithExtremes(size);
	}

	glPopMatrix();

	glEnable(GL_DEPTH_TEST);
}
