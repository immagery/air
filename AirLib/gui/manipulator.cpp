#include "manipulator.h"
#include <utils\utilGL.h>
#include <utils\util.h>

#define CURVE_RES 50

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

manipulator::manipulator()
{
	bModifierMode = false;
	size = 1.0;
	type = MANIP_NONE;
	axis = AXIS_X;
	bEnable = false;

	projectedPoint = Vector3d(0,0,0);

	mode = TM_HIERARCHY;
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

void manipulator::applyTransformation(object* obj, manipulatorType type, transformMode mode)
{
	if(mode == TM_HIERARCHY)
	{
		if(type == MANIP_MOVE)
			obj->setTranslation(currentframe.position.x(),currentframe.position.y(),currentframe.position.z(), false);

		if(type == MANIP_ROT)
			obj->setRotation(currentframe.rotation, false);
	}
	else if(mode == TM_SINGLE)
	{
		if(type == MANIP_MOVE)
			obj->setTranslation(currentframe.position.x(),currentframe.position.y(),currentframe.position.z(), true);
		
		if(type == MANIP_ROT)
			obj->setRotation(currentframe.rotation, true);	
	}

	if(obj->shading)
		obj->shading->dirtyFlag = true;

	obj->dirtyFlag = true;

	obj->update();
}

void manipulator::startManipulator(Vector3d& rayOrigin, Vector3d& rayDir, bool withOffset)
{
	orig = rayOrigin;
	dir = rayDir;
	previousframe = currentframe;

	if(type == MANIP_MOVE)
	{
		Vector3d u,v;
		Vector3d v1,v2;

		// Get directions
		if(axis == AXIS_X)
		{
			u = Vector3d(1,0,0);
		}
		else if(axis == AXIS_Y)
		{
			u = Vector3d(0,1,0);
		}
		else if(axis == AXIS_Z)
		{
			u = Vector3d(0,0,1);
		}
		
		u = currentframe.rotation._transformVector(u);
		v = u.cross(rayDir);
		v.normalize();

		Vector3d I; 
		int intersecFlag = intersect3D_RayPlane(rayOrigin, rayDir, previousframe.position,  v, u, I);
		if(intersecFlag != 1) 
			return; 

		// Whe save the offset for better user interaction.
		previousframe.posOffset = (I-previousframe.position).dot(u)*u;
		
		selectedPoint = previousframe.posOffset + previousframe.position;

	}
	else if(type == MANIP_ROT)
	{
		Vector3d u(0,0,0),v(0,0,0);

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
		
		u = previousframe.rotation._transformVector(u);
		v = previousframe.rotation._transformVector(v);

		u.normalize();
		v.normalize();

		Vector3d I; 
		int intersecFlag = intersect3D_RayPlane(rayOrigin, rayDir, previousframe.position,v,u, I);

		if(intersecFlag != 1) 
		{
			//printf("No intersecciona!\n");
			return;
		}

		selectedPoint = I;

		// Whe save the offset for better user interaction.
		previousframe.rotOffset.setFromTwoVectors(u, (previousframe.position-I).normalized());
		//printf("Setting transformation->selectedPoint: %f %f %f\n",selectedPoint.x(), selectedPoint.y(),selectedPoint.z());
	}
}

void manipulator::setManipulator(Vector3d& rayOrigin, Vector3d& rayDir, bool withOffset)
{
	orig = rayOrigin;
	dir = rayDir;
	previousframe = currentframe;

	if(type == MANIP_MOVE)
	{
		Vector3d u,v;
		Vector3d v1,v2;

		// Get directions
		if(axis == AXIS_X)
		{
			u = Vector3d(1,0,0);
		}
		else if(axis == AXIS_Y)
		{
			u = Vector3d(0,1,0);
		}
		else if(axis == AXIS_Z)
		{
			u = Vector3d(0,0,1);
		}
		
		u = currentframe.rotation._transformVector(u);
		v = u.cross(rayDir);
		v.normalize();

		Vector3d I; 
		int intersecFlag = intersect3D_RayPlane(rayOrigin, rayDir, previousframe.position,  v, u, I);
		if(intersecFlag != 1) 
			return; 

		selectedPoint = I;
	}
	else if(type == MANIP_ROT)
	{
		Vector3d u(0,0,0),v(0,0,0);

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
		
		u = previousframe.rotation._transformVector(u);
		v = previousframe.rotation._transformVector(v);

		u.normalize();
		v.normalize();

		Vector3d I; 
		int intersecFlag = intersect3D_RayPlane(rayOrigin, rayDir, previousframe.position,v,u, I);

		if(intersecFlag != 1) 
		{
			//printf("No intersecciona!\n");
			return;
		}

		selectedPoint = I;
		//printf("Setting transformation->selectedPoint: %f %f %f\n",selectedPoint.x(), selectedPoint.y(),selectedPoint.z());
	}
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
		}
		else if(axis == AXIS_Y)
		{
			u = Vector3d(0,1,0);
		}
		else if(axis == AXIS_Z)
		{
			u = Vector3d(0,0,1);
		}
		
		u = currentframe.rotation._transformVector(u);
		v = u.cross(rayDir);
		v.normalize();

		Vector3d I; 
		int intersecFlag = intersect3D_RayPlane(rayOrigin, rayDir, previousframe.position, v, u, I);

		if(intersecFlag != 1) 
			return;

		Vector3d applyIncrement = I - previousframe.position;
		applyIncrement = applyIncrement.dot(u.normalized())*u;

		currentframe.position = previousframe.position + applyIncrement - previousframe.posOffset;

		projectedPoint2 = previousframe.position + applyIncrement;
		projectedPoint = I;
	}
	else if(type == MANIP_ROT)
	{
		Vector3d u(0,0,0),v(0,0,0);

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
		
		u = previousframe.rotation._transformVector(u);
		v = previousframe.rotation._transformVector(v);

		u.normalize();
		v.normalize();

		// Compute projection
		Vector3d I; 
		int intersecFlag = intersect3D_RayPlane(rayOrigin, rayDir, previousframe.position, v, u, I);

		if(intersecFlag != 1) 
		{
			printf("No intersecciona!\n");
			return;
		}

		Vector3d v1, v2;
		v1 = (I-previousframe.position);
		v2 = (selectedPoint-previousframe.position);

		v1.normalize();
		v2.normalize();

		Quaterniond appliedRot;
		appliedRot.setFromTwoVectors(v2, v1);
		currentframe.rotation = appliedRot.normalized()*previousframe.rotation;
		projectedPoint = I;

		//float angle = acos(v2.dot(v1))/(v2.norm()*v1.norm())*360/(2*M_PI);
		//printf("rayOrigin: %f %f %f\n",rayOrigin.x(), rayOrigin.y(),rayOrigin.z());
		//printf("rayDir: %f %f %f\n",rayDir.x(), rayDir.y(),rayDir.z());
		//printf("angle: %f \n",angle);
		//printf("previousframe.position: %f %f %f\n",previousframe.position.x(), previousframe.position.y(),previousframe.position.z());
		//printf("selectedPoint: %f %f %f\n",selectedPoint.x(), selectedPoint.y(),selectedPoint.z());
		//printf("I: %f %f %f\n\n",I.x(), I.y(),I.z());
	}
	else if(type == MANIP_SCL)
	{

	}
}

void manipulator::drawFuncNames()
{
	if(!bEnable) return;

	glPushMatrix();

	Matrix4d transformMatrix;
	previousframe.getMatrix(transformMatrix);
	GLdouble multiplyingMatrix[16] = {transformMatrix(0,0), transformMatrix(0,1), 
											transformMatrix(0,2), transformMatrix(0,3),
										transformMatrix(1,0), transformMatrix(1,1), 
											transformMatrix(1,2), transformMatrix(1,3),
										transformMatrix(2,0), transformMatrix(2,1), 
											transformMatrix(2,2), transformMatrix(2,3),
										transformMatrix(3,0), transformMatrix(3,1), 
											transformMatrix(3,2), transformMatrix(3,3)};

	glMultMatrixd(multiplyingMatrix);

	if(type == MANIP_NONE)
	{
		// no manipulator visible
	}
	else if(type == MANIP_MOVE)
	{
		glDisable(GL_LIGHTING);

		glLineWidth(6);
		//glPushName(0);
		glPointSize(15);
		glBegin(GL_POINT);
		glColor3f(1.0,1.0,1.0);
		glVertex3d(0,0,0);
		glEnd();
		glPopName();

		glPushName(1);
		glBegin(GL_LINES);
		glColor3f(1.0,0,0);
		glVertex3d(0,0,0);
		glVertex3d(size*3,0,0);
		glEnd();
		glPopName();

		glPushName(2);
		glBegin(GL_LINES);
		glColor3f(0,1.0,0);
		glVertex3d(0,0,0);
		glVertex3d(0,size*3,0);
		glEnd();
		glPopName();

		glPushName(3);
		glBegin(GL_LINES);
		glColor3f(0,0,1.0);
		glVertex3d(0,0,0);
		glVertex3d(0,0,size*3);
		glEnd();
		glPopName();

		glLineWidth(1.0);

		glEnable(GL_LIGHTING);
	}
	else if(type == MANIP_ROT)
	{
		glLineWidth(8);
		glDisable(GL_LIGHTING);
		glPushMatrix();
		//glColor3f(1.0,0,0);

		glPushName(1.0);
		drawOpaqueCircle(CURVE_RES, size);
		glPopName();

		glRotatef(90, 0,1,0); // rotar en y
		//glColor3f(0,1.0,0);
		glPushName(3.0);
		drawOpaqueCircle(CURVE_RES,size);
		glPopName();
		glPopMatrix();

		glPushMatrix();
		//glColor3f(0,0,1.0); // rotar en x
		glRotatef(90, 0,0,1);
		glPushName(2.0);
		drawOpaqueCircle(CURVE_RES, size);
		glPopName();
		glPopMatrix();
		glEnable(GL_LIGHTING);
	}
	else if(type == MANIP_SCL)
	{
		drawAxisHandleWithExtremes(size);
	}

	glPopMatrix();
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
	//glDisable(GL_DEPTH_TEST);

	if(!bEnable) return;

	if(bModifierMode)
	{
		/// pintaríamos la forma en gris... dependediento del tipo de modificacion. 

		glPushMatrix();

		Matrix4d transformMatrix;
		previousframe.getMatrix(transformMatrix);
		GLdouble multiplyingMatrix[16] = {transformMatrix(0,0), transformMatrix(0,1), 
												transformMatrix(0,2), transformMatrix(0,3),
										  transformMatrix(1,0), transformMatrix(1,1), 
												transformMatrix(1,2), transformMatrix(1,3),
										  transformMatrix(2,0), transformMatrix(2,1), 
												transformMatrix(2,2), transformMatrix(2,3),
										  transformMatrix(3,0), transformMatrix(3,1), 
												transformMatrix(3,2), transformMatrix(3,3)
									};

		glMultMatrixd(multiplyingMatrix);

		int tempSize = (int)ceil(size);

		glPointSize(tempSize);
		glBegin(GL_POINTS);
		glColor3f(0.5,0.5,0.5);
		glVertex3d(0,0,0);
		glEnd();

		glPopMatrix();

		if(type == MANIP_MOVE)
		{
			glPointSize(1);
			glLineWidth(1);
			glBegin(GL_LINES);
			glColor3f(0.5,0.5,0.5);
			glVertex3d(projectedPoint2.x(),projectedPoint2.y(),projectedPoint2.z());
			glVertex3d(selectedPoint.x(),selectedPoint.y(),selectedPoint.z());
			glEnd();

			glPointSize(tempSize*8);
			glBegin(GL_POINTS);
			glColor3f(0.9,0.5,0.5);
			glVertex3d(projectedPoint2.x(),projectedPoint2.y(),projectedPoint2.z());
			glColor3f(0.9,0.5,0.9);
			glVertex3d(selectedPoint.x(),selectedPoint.y(),selectedPoint.z());
			//glColor3f(0.5,0.5,0.9);
			//glVertex3d(projectedPoint.x(),projectedPoint.y(),projectedPoint.z());
			glEnd();
		}
		else if(type == MANIP_ROT)
		{
			Vector3d v1, v2;

			v1 = (selectedPoint - previousframe.position);
			v2 = (projectedPoint - previousframe.position);

			Vector3d pt1 = previousframe.position+v1.normalized()*size;
			Vector3d pt2 = previousframe.position+v2.normalized()*size;

			if(pt2.norm() > 0)
				int parar = 0;


			glPointSize(tempSize*2);
			glDisable(GL_LIGHTING);
			glBegin(GL_LINES);
			glColor3f(0.7,0.7,0.7);
			glVertex3d(previousframe.position.x(),previousframe.position.y(),previousframe.position.z());
			glVertex3d(pt1.x(),pt1.y(),pt1.z());

			glVertex3d(previousframe.position.x(),previousframe.position.y(),previousframe.position.z());
			glVertex3d(pt2.x(),pt2.y(),pt2.z());
			glEnd();

			glBegin(GL_POINTS);
			glColor3f(1,1,1);
			glVertex3d(pt1.x(),pt1.y(),pt1.z());
			glVertex3d(previousframe.position.x(),previousframe.position.y(),previousframe.position.z());
			glVertex3d(pt2.x(),pt2.y(),pt2.z());
			glEnd();

			glEnable(GL_LIGHTING);
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

	int tempSize = (int) ceil(size);

	if(type == MANIP_NONE)
	{
		// no manipulator visible
	}
	else if(type == MANIP_MOVE)
	{
		glDisable(GL_LIGHTING);
		glLineWidth(tempSize);
		
		if(axis== AXIS_X)
			glLineWidth(5);
		else
			glLineWidth(3);

		glBegin(GL_LINES);
		glColor3f(1.0,0,0);
		glVertex3d(0,0,0);
		glVertex3d(size*3,0,0);
		glEnd();

		if(axis== AXIS_Y)
			glLineWidth(5);
		else
			glLineWidth(3);

		glBegin(GL_LINES);
		glColor3f(0,1.0,0);
		glVertex3d(0,0,0);
		glVertex3d(0,size*3,0);
		glEnd();

		if(axis== AXIS_Z)
			glLineWidth(5);
		else
			glLineWidth(3);

		glBegin(GL_LINES);
		glColor3f(0,0,1.0);
		glVertex3d(0,0,0);
		glVertex3d(0,0,size*3);
		glEnd();

		if(axis== AXIS_VIEW)
			glLineWidth(5);
		else
			glLineWidth(3);

		glPointSize(8);
		glBegin(GL_POINTS);
		glColor3f(1,1,0);
		glVertex3d(0,0,0);
		glEnd();

		glLineWidth(1.0);

		/*
		glPointSize(3);
		glBegin(GL_POINTS);
		glColor3f(1,1,1);
		glVertex3d(selectedPoint.x(),selectedPoint.y(),selectedPoint.z());
		glColor3f(0,1,0);
		glVertex3d(projectedPoint.x(),projectedPoint.y(),projectedPoint.z());
		glEnd();
		*/

		glEnable(GL_LIGHTING);
	}
	else if(type == MANIP_ROT)
	{

		if(axis== AXIS_X)
			glLineWidth(tempSize);
		else
			glLineWidth(tempSize*2);

		glDisable(GL_LIGHTING);
		glPushMatrix();
		glColor3f(1.0,0,0);
		drawCircle(CURVE_RES, size);

		if(axis== AXIS_Z)
			glLineWidth(tempSize);
		else
			glLineWidth(tempSize*2);

		glRotatef(90, 0,1,0); // rotar en y
		glColor3f(0,0.0,1.0);
		drawCircle(CURVE_RES, size);
		glPopMatrix();

		if(axis== AXIS_Y)
			glLineWidth(tempSize);
		else
			glLineWidth(tempSize*2);

		glPushMatrix();
		glColor3f(0,1.0,0.0); // rotar en x
		glRotatef(90, 0,0,1);
		drawCircle(CURVE_RES, size);
		glPopMatrix();
		glEnable(GL_LIGHTING);
	}
	else if(type == MANIP_SCL)
	{
		drawAxisHandleWithExtremes(size);
	}

	glPopMatrix();

	//glEnable(GL_DEPTH_TEST);
}
