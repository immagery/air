#include "BulgeDeformer.h"

#include <DataStructures\AirRig.h>
#include <DataStructures\Geometry.h>
#include <DataStructures\Modelo.h>


wireDeformer::wireDeformer()
{
	W.clear();
	R.clear();
	r = 1.0; // radius
	s = 1.0; // scale

	lenght = 1.0;
	high = 1.0;

	orient = Quaterniond::Identity();
	pos = Vector3d(0,0,0);
}

void wireDeformer::init(Vector3d dir, Vector3d n, vector<Vector3d>& curvePointsini, vector<Vector3d>& curveCoords, float maxPressureChild)
{
	vector<Vector3d> curvePoints2;
	int firstNode = 0;
	curvePoints2.push_back(curvePointsini[firstNode]);
	firstNode++;

	for(int i = firstNode; i< curvePointsini.size(); i++)
	{
		Vector3d displ = curvePointsini[i]-curvePoints2.back();
		if(displ.dot(displ) > 0.03)
		{
			curvePoints2.push_back(curvePointsini[i]);
		}
	}

	/*
	vector<Vector3d> curvePoints2(curvePoints.size());
	for(int i = 0; i< curvePoints.size(); i++)
		curvePoints2[i] = curvePoints[i];

	vector<Vector3d> curveCoords2(curveCoords.size());
	for(int i = 0; i< curveCoords.size(); i++)
		curveCoords2[i] = curveCoords[i];

	int firstNode = 0;

	
	// Clean vectors
	curvePoints.clear();
	curveCoords.clear();

	// Push the first node
	curveCoords.push_back(curveCoords2[firstNode]);
	curvePoints.push_back(curvePoints2[firstNode]);
	
	firstNode++;

	// Push nodes if the distance is big enough
	for(int i = firstNode; i< curvePoints2.size(); i++)
	{
		Vector3d displ = curvePoints2[i]-curvePoints.back();
		if(displ.dot(displ) > 0.03)
		{
			curvePoints.push_back(curvePoints2[i]);
			curveCoords.push_back(curveCoords2[i]);
		}
	}
	curvePoints2.clear();
	*/

	pos = curvePoints2.front();
	mDir = (curvePoints2.back()-pos).normalized();
	mN = n.normalized(); 

	Quaterniond rot1, rot2;
	rot1.setFromTwoVectors(Vector3d(1,0,0), mDir);

	Vector3d newY = rot1._transformVector(Vector3d(0,1,0));
	rot2.setFromTwoVectors(newY.normalized(), mN);

	orient = rot2*rot1;

	Vector3d traslatedPoint = curvePoints2.back()-pos;
	float totalProjX = traslatedPoint.dot(mDir);

	if(curvePoints2.size() <= 3)
	{
		R.clear();
		W.clear();

		W.addPoint(0.0, 0.0);
		W.addPoint(0.5, 0.0);
		W.addPoint(1.0, 0.0);

		R.addPoint(0.0, 0.0);
		R.addPoint(0.5, 0.0);
		R.addPoint(1.0, 0.0);

		return;
	}

	R.clear();
	for(int cvPoint = 0; cvPoint<curvePoints2.size(); cvPoint++)
	{
		Vector3d traslatedPoint = curvePoints2[cvPoint]-pos;
		float projX = traslatedPoint.dot(mDir)/totalProjX;
		float projY = traslatedPoint.dot(mN)/totalProjX;
		R.addPoint(projX, projY);
	}

	Spline bulgeCurve;
	bulgeCurve.clear();
	bulgeCurve.addPoint(0.0, 0.0);
	bulgeCurve.addPoint(0.1, maxPressureChild/10);
	bulgeCurve.addPoint(0.40, maxPressureChild/50);
	bulgeCurve.addPoint(0.50, 0.0);
	bulgeCurve.addPoint(0.60, 0.0);
	bulgeCurve.addPoint(1.0, 0.0);

	int parts = curvePoints2.size();
	W.clear();
	for(int cvPoint =0; cvPoint<=parts; cvPoint++)
	{
		float pty = R(((float)cvPoint/(float)parts));
		float bulgeOffset = bulgeCurve(((float)cvPoint/(float)parts));
		W.addPoint(((float)cvPoint/(float)parts), pty+bulgeOffset);
	}
}

float wireDeformer::implicitFunction(float value)
{
	if(value >= r) return 0;
	if(value <= 0) return 1;

	// Compute weights funtion depending on the distance.
	// Maybe I can reuse the function implemented early for
	// model weights.

	//computeWeightProportional()

	// now, just a linear ramp
	return 1-(value/r);
}

void wireDeformer::getDisplacement(Vector3d pt, float u, float w, Vector3d& displ)
{
	// De momento calculamos los puntos así, pero ya haremos el cambio a puntos 3D.
	Vector3d Wproj = Vector3d(u*lenght, W(u)*high, 0);
	Vector3d Rproj = Vector3d(u*lenght, R(u)*high, 0);

	// Cambiamos base
	Vector3d ptOrig = pt - pos; // traslacion
	ptOrig = orient.inverse()._transformVector(ptOrig); // rotacion

	// Aplicacion del escalado-> anulado si s = 1
	Vector3d ptScaled = Rproj + (ptOrig - Rproj)*(1+(s-1)*w);

	// Aplicacion de la rotacion -> por ahora lo paso.
	Vector3d ptRotated = ptScaled;

	// Aplicacion de la traslacion
	Vector3d ptDef = ptRotated + (Wproj-Rproj) * w;

	// devolvemos el punto a la base correcta
	displ = (orient._transformVector(ptDef) + pos)-pt;
}

BulgeGroup::BulgeGroup()
{
	// Curve wire deformers
	defCurve = NULL;

	deformerId = -1;
	childDeformerId = -1;

	direction = false;
}

BulgeDeformer::BulgeDeformer()
{
	enabled = false;
}

void projectPointOnPlane(Vector3d O, Vector3d pN, Vector3d dirX, Vector3d pt, Vector2d& ppt )
{
	Vector3d dirY = dirX.cross(pN);
	Vector3d tempPt = pt-((pt-O).dot(pN)*pN);
	ppt[0] = (tempPt-O).dot(dirX);
	ppt[1] = (tempPt-O).dot(dirY);
}

void projectTriangleOnPlane(Vector3d O, Vector3d pN, Vector3d dirX, Vector3d& pt1, Vector3d& pt2, Vector3d& pt3, vector<Vector2d>& ppts )
{
	ppts.resize(3);

	vector<Vector3d> in_pts;
	in_pts[0] = pt1;
	in_pts[1] = pt2;
	in_pts[2] = pt3;

	Vector3d dirY = dirX.cross(pN);

	for(int i = 0; i< 3; i++)
	{
		Vector3d tempPt = in_pts[i]-((in_pts[i]-O).dot(pN)*pN);
		ppts[i][0] = (tempPt-O).dot(dirX);
		ppts[i][1] = (tempPt-O).dot(dirY);
	}
}

enum intersecionTypes{INT_VERTEX = 0, INT_EDGE, INT_INSIDE, INT_NO_INT};

bool getNextTriangleIdxEdge(vector<GraphNodePolygon*>& triangleGroup, int& triangleId, int& idx1, int& idx2)
{
	int nodeId1 = triangleGroup[triangleId]->verts[idx1]->id;
	int nodeId2 = triangleGroup[triangleId]->verts[idx2]->id;

	for(int i = 0; i < triangleGroup.size(); i++)
	{
		GraphNodePolygon* tr = triangleGroup[i];
		if(i == triangleId) continue;

		for(int edgeIdx = 0; edgeIdx < 3; edgeIdx++)
		{
			if(tr->verts[edgeIdx]->id == nodeId1 && tr->verts[(edgeIdx+1)%3]->id == nodeId2)
			{
				idx1 = edgeIdx;
				idx2 = (edgeIdx+1)%3;
				triangleId = i;
				return true;
			}
			else if(tr->verts[edgeIdx]->id == nodeId2 && tr->verts[(edgeIdx+1)%3]->id == nodeId1)
			{
				idx1 = edgeIdx;
				idx2 = (edgeIdx+1)%3;
				triangleId = i;
				return true;
			}
		}
	}

	return false;
}

bool getNextTriangleIdxFromDirection(vector<GraphNodePolygon*>& triangleGroup, int& triangleId, int& currentIntersectionIdx, Vector3d D)
{
	int nodeId = triangleGroup[triangleId]->verts[currentIntersectionIdx]->id;
	float maxDots = -1;
	int maxDotsIdx = -1;
	int intIdx = -1;

	Vector3d dir = D - triangleGroup[triangleId]->verts[currentIntersectionIdx]->position;
	for(int i = 0; i < triangleGroup.size(); i++)
	{
		GraphNodePolygon* tr = triangleGroup[i];
		if(i == triangleId) continue;

		if(tr->verts[0]->id == nodeId) 
		{
			vector<Vector3d> v(3);
			//v[0] = tr->verts[1]->position-tr->verts[0]->position;
			v[1] = tr->verts[2]->position-tr->verts[1]->position;
			v[2] = tr->verts[0]->position-tr->verts[2]->position;
			float dots = v[1].dot(dir) + v[2].dot(dir);
			if(dots>maxDots)
			{
				maxDots = dots;
				maxDotsIdx = i;
				intIdx = 0;
			}
		}
		else if(tr->verts[1]->id == nodeId)
		{
			vector<Vector3d> v(3);
			v[0] = tr->verts[1]->position-tr->verts[0]->position;
			//v[1] = tr->verts[2]->position-tr->verts[1]->position;
			v[2] = tr->verts[0]->position-tr->verts[2]->position;
			float dots = v[0].dot(dir) + v[2].dot(dir);
			if(dots>maxDots)
			{
				maxDots = dots;
				maxDotsIdx = i;
				intIdx = 1;
			}
		}
		else if(tr->verts[2]->id == nodeId)
		{
			vector<Vector3d> v(3);
			v[0] = tr->verts[1]->position-tr->verts[0]->position;
			v[1] = tr->verts[2]->position-tr->verts[1]->position;
			//v[2] = tr->verts[0]->position-tr->verts[2]->position;
			float dots = v[0].dot(dir) + v[1].dot(dir);
			if(dots>maxDots)
			{
				maxDots = dots;
				maxDotsIdx = i;
				intIdx = 2;
			}
		}
	}

	if(maxDotsIdx > 0)
	{
		triangleId = maxDotsIdx;
		currentIntersectionIdx = intIdx;
		return true;
	}
	else
		return false;
}

void getIntersecction2D(Vector2d vert, Vector2d edgeDir, Vector2d& v, Vector2d dir)
{
	float a1 = dir.y()/dir.x();
	float a2 = edgeDir.y()/edgeDir.x();

	float b1 = v.y()-a1*v.x();
	float b2 = vert.y()-a2*vert.x();

	float x = (b2-b1)/(a1-a2);
	float y = a1*x+b1;

	v = Vector2d(x,y);
}

bool get2DBarCoords(Vector2d pt0, Vector2d pt1, Vector2d pt2, Vector2d pt, Vector3d& coords)
{
	Vector2d A,B,C;
	A = pt0; B = pt1; C = pt2;
	double AreaTotal = (A.x()*(B.y()-C.y())+B.x()*(C.y()-A.y())+C.x()*(A.y()-B.y()))/2;

	A = pt; B = pt0; C = pt1;
	double a0 = (A.x()*(B.y()-C.y())+B.x()*(C.y()-A.y())+C.x()*(A.y()-B.y()))/2;

	A = pt; B = pt1; C = pt2;
	double a1 = (A.x()*(B.y()-C.y())+B.x()*(C.y()-A.y())+C.x()*(A.y()-B.y()))/2;

	A = pt; B = pt2; C = pt0;
	double a2 = (A.x()*(B.y()-C.y())+B.x()*(C.y()-A.y())+C.x()*(A.y()-B.y()))/2;

	// Interseccion por el centro
	double a = a1/AreaTotal;
	double b = a2/AreaTotal;
	double c = a0/AreaTotal;

	double sum = a+b+c;

	coords = Vector3d(a/sum,b/sum,c/sum);

	//assert(fabs(a0+a1+a2 - AreaTotal) < SMALL_NUM);
	
	return true;
}

bool get3DBarCoords(Vector3d  pt0, Vector3d pt1, Vector3d pt2, Vector3d pt, Vector3d& coords )
{
	Vector3d v1 = pt1-pt0;
	Vector3d v2 = pt2-pt0;
	Vector3d n = v1.cross(v2).normalized();

	// Pasamos a 2D y lanzamos el calculo que ya conocemos que funciona.
	Vector3d eje1 = v1.normalized();
	Vector3d eje2 = (n.cross(v1)).normalized();

	Vector2d pt2d_1(0,0);
	Vector2d pt2d_2(v1.dot(eje1),v1.dot(eje2));
	Vector2d pt2d_3(v2.dot(eje1),v2.dot(eje2));
	Vector2d pt2d((pt-pt0).dot(eje1),(pt-pt0).dot(eje2));

	return get2DBarCoords(pt2d_1,pt2d_2,pt2d_3, pt2d, coords);
}

void getCurvePoints(Vector3d O, int OType, Vector3d D, 
					Vector3d pN, Vector3d dirX, 
					vector<GraphNodePolygon*>& triangleGroup, 
					vector< vector<int> >& triVertMap, 
					vector<Vector3d>& points, 
					vector<Vector3d>& coords, 
					vector<int>& triangleSequence,
					int firstTri, int lastTri)
{
	// Ponemos el origen
	
	Vector2d current(0,0);
	projectPointOnPlane(O, pN, dirX, O, current);
	int currentIntersectionType = OType;
	int currentIntersectionIdx = 0;

	Vector2d fin(0,0); 
	projectPointOnPlane(O, pN, dirX, D, fin);

	Vector2d dir = -(fin-current).normalized();

	int currentTri = firstTri;

	// First Step
	vector<Vector2d> ptsTemp;
	ptsTemp.resize(3);
	// Project the triangle
	projectPointOnPlane(O, pN, dirX, triangleGroup[currentTri]->verts[0]->position, ptsTemp[0]);
	projectPointOnPlane(O, pN, dirX, triangleGroup[currentTri]->verts[1]->position, ptsTemp[1]);
	projectPointOnPlane(O, pN, dirX, triangleGroup[currentTri]->verts[2]->position, ptsTemp[2]);

	// The first 3d point
	//points.push_back(O);

	while(currentTri != lastTri)
	{
		// Project the triangle over the plane
		vector<Vector2d> pts(3);
		projectPointOnPlane(O, pN, dirX, triangleGroup[currentTri]->verts[0]->position, pts[0]);
		projectPointOnPlane(O, pN, dirX, triangleGroup[currentTri]->verts[1]->position, pts[1]);
		projectPointOnPlane(O, pN, dirX, triangleGroup[currentTri]->verts[2]->position, pts[2]);

		// Get the direction
		dir = -(fin-current).normalized();

		// Save the current computation indices
		int computationTri = currentTri;
		Vector2d computationCurrent = current;

		if(currentIntersectionType == INT_VERTEX) // If it's a vertex
		{
			// Obtener vertices
			int vert0 = currentIntersectionIdx;
			int vert1 = (currentIntersectionIdx+1)%3;
			int vert2 = (currentIntersectionIdx+2)%3;

			vector<Vector2d> v(3);
			v[0] = pts[vert1]-pts[vert0];
			v[1] = pts[vert2]-pts[vert1];
			v[2] = pts[vert0]-pts[vert2];

			if(fabs(v[0].dot(dir)-1) < SMALL_NUM) // Si la continuacion esta alineada con un vertice
			{
				currentIntersectionType = INT_VERTEX;
				// Ver que triangulo es el siguiente...
				current = pts[vert1];
				getNextTriangleIdxFromDirection(triangleGroup, currentTri,currentIntersectionIdx, D);
			}
			else if(fabs((-v[2]).dot(dir)-1) < SMALL_NUM) // Si la continuacion esta alineada con un vertice
			{
				currentIntersectionType = INT_VERTEX;
				// Ver que triangulo es el siguiente...
				current = pts[vert2];
				getNextTriangleIdxFromDirection(triangleGroup, currentTri,currentIntersectionIdx, D);
			}
			else // Si la continuacion esta alineada la arista contraria
			{
				currentIntersectionType = INT_EDGE;
				Vector2d n = Vector2d(-v[1].y(), v[1].x());
				current = pts[vert0]-n.dot(v[2])*n;
				int currentIntersectionIdx = vert1;
				int idx2 = vert2;
				getNextTriangleIdxEdge(triangleGroup, currentTri, currentIntersectionIdx, idx2);
			}
		}
		else if (currentIntersectionType == INT_EDGE)
		{
			// Obtener vertices
			int vert0 = currentIntersectionIdx;
			int vert1 = (currentIntersectionIdx+1)%3;
			int vert2 = (currentIntersectionIdx+2)%3;

			vector<Vector2d> v;
			v.resize(3);
			v[0] = pts[vert1]-pts[vert0];
			v[1] = pts[vert2]-pts[vert1];
			v[2] = pts[vert0]-pts[vert2];

			int a1 = (currentIntersectionIdx+1)%3;
			int a2 = (currentIntersectionIdx+2)%3;

			Vector2d n2(-v[1].y(), v[1].x());
			Vector2d n3(-v[2].y(), v[2].x());

			Vector2d middle = pts[a2]-current;

			if(fabs(middle.dot(dir)-1) < SMALL_NUM) // Si la continuacion esta alineada con un vertice
			{
				currentIntersectionType = INT_VERTEX;
				// Ver que triangulo es el siguiente...
				current = pts[vert2];
				currentIntersectionIdx = vert2;
				getNextTriangleIdxFromDirection(triangleGroup, currentTri,currentIntersectionIdx, D);
			}
			else
			{
				Vector2d nMiddle(-middle.y(), middle.x());

				if(nMiddle.dot(dir)>0) // Primera arista
				{
					currentIntersectionType = INT_EDGE;
					// Ver que triangulo es el siguiente...
					getIntersecction2D(pts[vert0], -v[2].normalized(), current, dir);
					int idx2 = vert2;
					currentIntersectionIdx = vert0;
					getNextTriangleIdxEdge(triangleGroup, currentTri, currentIntersectionIdx, idx2);
				}
				else // Segunda arista
				{
					currentIntersectionType = INT_EDGE;
					// Ver que triangulo es el siguiente...
					getIntersecction2D(pts[vert1],v[1].normalized(), current, dir);
					currentIntersectionIdx = vert1;
					int idx2 = vert2;
					getNextTriangleIdxEdge(triangleGroup, currentTri, currentIntersectionIdx, idx2);
				}
			}
		}
		else if (currentIntersectionType == INT_INSIDE)
		{

			// Obtener vertices
			int vert0 = currentIntersectionIdx;
			int vert1 = (currentIntersectionIdx+1)%3;
			int vert2 = (currentIntersectionIdx+2)%3;

			vector<Vector2d> v(3);
			v[0] = pts[vert1]-pts[vert0];
			v[1] = pts[vert2]-pts[vert1];
			v[2] = pts[vert0]-pts[vert2];

			Vector2d n0(-v[0].y(), v[0].x());
			Vector2d n1(-v[1].y(), v[1].x());
			Vector2d n2(-v[2].y(), v[2].x());

			float nv1, nv2, nv3;

			if(dir.dot(n0)>=0) // Arista que no corta
			{
				nv1 = vert0;
				nv2 = vert1;
				nv3 = vert2;
			}
			else if(dir.dot(n1) >= 0)
			{
				nv1 = vert1;
				nv2 = vert2;
				nv3 = vert0;
			}
			else if(dir.dot(n2) >= 0)
			{
				nv1 = vert2;
				nv2 = vert0;
				nv3 = vert1;
			}

			v[0] = (pts[nv2]-pts[nv1]).normalized();
			v[1] = (pts[nv3]-pts[nv2]).normalized();
			v[2] = (pts[nv1]-pts[nv3]).normalized();

			Vector2d middle = (pts[nv3]-current).normalized();
			if(fabs(middle.dot(dir)-1) < SMALL_NUM) // Si la continuacion esta alineada con un vertice
			{
				currentIntersectionType = INT_VERTEX;
				// Ver que triangulo es el siguiente...
				current = pts[nv2];
				currentIntersectionIdx = nv2;
				getNextTriangleIdxFromDirection(triangleGroup, currentTri,currentIntersectionIdx, D);
			}
			else 
			{
				Vector2d nMiddle(-middle.y(), middle.x());
				if(nMiddle.dot(dir)>0) // Primera arista
				{
					currentIntersectionType = INT_EDGE;
					// Ver que triangulo es el siguiente...
					getIntersecction2D(pts[nv1], -v[2].normalized(), current, dir);
					int idx2 = nv3;
					currentIntersectionIdx = nv1;
					getNextTriangleIdxEdge(triangleGroup, currentTri, currentIntersectionIdx, idx2);
				}
				else
				{
					currentIntersectionType = INT_EDGE;
					// Ver que triangulo es el siguiente...
					getIntersecction2D(pts[nv2],v[1].normalized(), current, dir);
					currentIntersectionIdx = nv2;
					int idx2 = nv3;
					getNextTriangleIdxEdge(triangleGroup, currentTri, currentIntersectionIdx, idx2);
				}
			}
		}

		coords.push_back(Vector3d(0.3,0.3,0.3));
		get2DBarCoords(pts[0], pts[1], pts[2], computationCurrent, coords.back());
		
		Vector3d newPoint = triangleGroup[computationTri]->verts[0]->position*coords.back().x()+
							triangleGroup[computationTri]->verts[1]->position*coords.back().y()+
							triangleGroup[computationTri]->verts[2]->position*coords.back().z();

		triangleSequence.push_back(computationTri);
		points.push_back(newPoint);
	}

	// Calculamos las coordenadas y datos del punto final
	vector<Vector2d> pts;
	pts.resize(3);
	// Project the triangle
	projectPointOnPlane(O, pN, dirX, triangleGroup[lastTri]->verts[0]->position, pts[0]);
	projectPointOnPlane(O, pN, dirX, triangleGroup[lastTri]->verts[1]->position, pts[1]);
	projectPointOnPlane(O, pN, dirX, triangleGroup[lastTri]->verts[2]->position, pts[2]);

	coords.push_back(Vector3d(0.3,0.3,0.3));
	get2DBarCoords(pts[0], pts[1], pts[2], fin, coords.back());

	triangleSequence.push_back(lastTri);
	points.push_back(D);
}

bool getRayTriangle_Intersection(Vector3d O,Vector3d dir, Vector3d pt0, Vector3d pt1, Vector3d pt2, Vector3d& I, int& type, Vector3d& coords, bool enableBarCoords)
{
	Vector3d v1 = pt1-pt0;
	Vector3d v2 = pt2-pt0;
	Vector3d n = v1.cross(v2).normalized();

	Vector3d n1 = v1.cross(n).normalized();
	Vector3d n2 = (pt2-pt1).cross(n).normalized();
	Vector3d n3 = (pt0-pt2).cross(n).normalized();

	coords = Vector3d(-1,-1,-1);

	if(n.dot(dir) >= 0)
	{
		intersect3D_RayPlane(O, dir, pt0, v1, n1, I);
		int count = 0;
		int count2 = 0;
		bool cond[3] = {false, false, false};

		float value = (I-pt0).dot(n1);
		if(value > 0)count++;	

		if(fabs(value) < SMALL_NUM)
		{
			cond[0] = true;
			count2++;
		}
			
		value = (I-pt1).dot(n2);
		if(value > 0)count++;	

		if(fabs(value) < SMALL_NUM)
		{
			cond[1] = true;
			count2++;
		}

		value = (I-pt2).dot(n3);
		if(value > 0) count++;

		if(fabs(value) < SMALL_NUM)
		{
			cond[2] = true;
			count2++;
		}
		
			
		if(count == 3 || count == 0)
		{
			// Ha habido interseccion correcta.

			// Hacemos una inversión
			//if(count == 0) for(int condIdx = 0; condIdx < 3; condIdx++) cond[0] = !cond[0];

			//if(enableBarCoords) // calculamos coordenadas de referencia
			{
				if(count2 == 2)
				{
					type = INT_VERTEX;

					// tenemos interseccion en un vertice
					if(!cond[0]) coords = Vector3d(-1,-1, 1);
					else if(!cond[1]) coords = Vector3d(1,-1,-1);
					else if(!cond[2]) coords = Vector3d(-1,1,-1);
				}
				else if(count2 == 1)
				{
					type = INT_EDGE;

					// Interseccion en una arista
					if(cond[0])
					{
						float val = (I-pt0).norm()/(pt1-pt0).norm();
						coords = Vector3d(1-val,val,-1);
					}
					else if(cond[1])
					{
						float val = (I-pt1).norm()/(pt2-pt1).norm();
						coords = Vector3d(-1,1-val,val);
					}
					else if(cond[2])
					{
						float val = (I-pt2).norm()/(pt0-pt2).norm();
						coords = Vector3d(val,-1,1-val);
					}
				}
				else if(count2 == 0)
				{
					type = INT_INSIDE;
				
					get3DBarCoords(pt0, pt1, pt2, I, coords);

					// Interseccion por el centro
					/*float a = 1-((I-pt1).dot(n2)/(pt0-pt1).dot(n2));
					float b = 1-((I-pt2).dot(n3)/(pt1-pt2).dot(n3));
					float c = 1-((I-pt0).dot(n1)/(pt2-pt0).dot(n1));

					float sum = a+b+c;
					if(sum > 0) coords = Vector3d(a/sum,b/sum,c/sum);

					assert(a>= 0 && b>= 0 && c>= 0 && sum> 0);*/
				}
			}

			return true;
		}
	}

	return false;
}

int getRayProjectionCoords(Vector3d origin,Vector3d ray, Geometry* m, vector<GraphNodePolygon*> triangleGroup, Vector3d& projectionCoords, int& type)
{
	// Buscamos el primer triangulo que encontremos que este interseccionado.
	for(int i = 0; i< triangleGroup.size(); i++)
	{
		Vector3d pt0 = triangleGroup[i]->verts[0]->position;
		Vector3d pt1 = triangleGroup[i]->verts[1]->position;
		Vector3d pt2 = triangleGroup[i]->verts[2]->position;

		Vector3d projection;
		if(getRayTriangle_Intersection(origin, ray, pt0, pt1, pt2, projection, type, projectionCoords, true))
			return i;
	}

	return -1;
}

int getRayProjection(Vector3d planeOrigin,Vector3d bisectriz, Geometry* m, vector<GraphNodePolygon*> triangleGroup, Vector3d& projection, int& type)
{
	// Buscamos el primer triangulo que encontremos que este interseccionado.
	for(int i = 0; i< triangleGroup.size(); i++)
	{
		Vector3d pt0 = triangleGroup[i]->verts[0]->position;
		Vector3d pt1 = triangleGroup[i]->verts[1]->position;
		Vector3d pt2 = triangleGroup[i]->verts[2]->position;

		Vector3d coords;
		if(getRayTriangle_Intersection(planeOrigin, bisectriz, pt0, pt1, pt2, projection, type, coords, false))
			return i;
	}

	return -1;
}

void refineParameter(int vertId, int minDistId, float minDistance001, vector<float>& distances, vector<float>& uParameter, vector<Vector3d>& refPoints, vector<GraphNodePolygon*>& triangleGroup, vector<int>& triangleSequence, symMatrix& BihDistances, float& newDist, float& newUParam)
{
	int idxIni, idxFin, idxCenter;
	int triIni, triFinal;
	float uini, ufin;
	Vector3d vini, vfin;

	vector<Vector3d> candidates;
	vector<float> uCandidates;
	vector<float> dCandidates; 
	vector<int> trianglesToTest;

	int subdivionParts = 10;

	if(refPoints.size() == 0)  return;

	if(refPoints.size() == 1)
	{
		// No hay nada que decidir
		newDist = minDistance001;
		newUParam = uParameter[minDistId];
		return; 
	}
	if(refPoints.size() == 2)
	{
		vini = refPoints[0];
		vini = refPoints[1];

		uini = uParameter[0];
		ufin = uParameter[1];

		candidates.resize(subdivionParts+1);
		uCandidates.resize(subdivionParts+1);
		dCandidates.resize(subdivionParts+1);
		trianglesToTest.resize(subdivionParts+1);

		Vector3d dir = vfin-vini;
		float incrU = ufin-uini;

		for(int i = 0; i<= subdivionParts; i++)
		{
			candidates[i] = vini + (dir*(float)i/(float)subdivionParts);
			uCandidates[i] = uini + (incrU*(float)i/(float)subdivionParts);
			trianglesToTest[i] = triangleSequence[1];
		}

		dCandidates[0] = distances[0];
		dCandidates[dCandidates.size()-1] = distances[1];
	}
	else
	{
		// Si estamos en el inicio
		if(minDistId == 0)
		{
			idxIni = minDistId;
			idxCenter = minDistId+1;
			idxFin = minDistId+2;

			triIni = idxIni;
			triFinal = idxCenter;
		}
		else if(minDistId == refPoints.size()-1)	
		{
			idxIni = minDistId-2;
			idxCenter = minDistId-1;
			idxFin = minDistId;

			triIni = idxIni;
			triFinal = idxCenter;
		}
		else
		{
			idxIni = minDistId-1;
			idxCenter = minDistId;
			idxFin = minDistId+1;

			triIni = idxIni;
			triFinal = idxCenter;
		}

	
			uini = uParameter[idxIni];
			ufin = uParameter[idxCenter];
			
			vini = refPoints[idxIni];
			vfin = refPoints[idxCenter];

			candidates.resize(subdivionParts*2+1);
			uCandidates.resize(subdivionParts*2+1);
			dCandidates.resize(subdivionParts*2+1);
			trianglesToTest.resize(subdivionParts*2+1);

			Vector3d dir = vfin-vini;
			float incrU = ufin-uini;

			for(int i = 0; i<= subdivionParts; i++)
			{
				candidates[i] = vini + (dir*(float)i/(float)subdivionParts);
				uCandidates[i] = uini + (incrU*(float)i/(float)subdivionParts);
				trianglesToTest[i] = triangleSequence[triIni];
			}

			uini = uParameter[idxCenter];
			ufin = uParameter[idxFin];
			
			vini = refPoints[idxCenter];
			vfin = refPoints[idxFin];

			dir = vfin-vini;
			incrU = ufin-uini;

			for(int i = 1; i<= subdivionParts; i++)
			{
				candidates[i+subdivionParts] = vini + (dir*(float)i/(float)subdivionParts);
				uCandidates[i+subdivionParts] = uini + (incrU*(float)i/(float)subdivionParts);
				trianglesToTest[i+subdivionParts] = triangleSequence[triFinal];
			}

			dCandidates[0] = distances[idxIni];
			dCandidates[subdivionParts] = distances[idxCenter];
			dCandidates[dCandidates.size()-1] = distances[idxFin];

		/*
			uini = uParameter[idxIni];
			ufin = uParameter[idxCenter];

			vini = refPoints[idxIni];
			vfin = refPoints[idxCenter];

			candidates.resize(subdivionParts+1);
			uCandidates.resize(subdivionParts+1);
			dCandidates.resize(subdivionParts+1);
			trianglesToTest.resize(subdivionParts+1);

			Vector3d dir = vfin-vini;
			float incrU = ufin-uini;

			for(int i = 0; i<= subdivionParts; i++)
			{
				candidates[i] = vini + (dir*(float)i/(float)subdivionParts);
				uCandidates[i] = uini + (incrU*(float)i/(float)subdivionParts);
				trianglesToTest[i] = triangleSequence[idxIni];
			}

			dCandidates[0] = distances[idxIni];
			dCandidates[dCandidates.size()-1] = distances[idxFin];

		}

		// Si estamos en el final
		else if(minDistId >= refPoints.size()-1)	
		{
			idxIni = minDistId-1;
			idxFin = minDistId;

			uini = uParameter[idxIni];
			ufin = uParameter[idxFin];

			vini = refPoints[idxIni];
			vfin = refPoints[idxFin];

			candidates.resize(subdivionParts+1);
			uCandidates.resize(subdivionParts+1);
			dCandidates.resize(subdivionParts+1);
			trianglesToTest.resize(subdivionParts+1);

			Vector3d dir = vfin-vini;
			float incrU = ufin-uini;

			for(int i = 0; i<= subdivionParts; i++)
			{
				candidates[i] = vini + (dir*(float)i/(float)subdivionParts);
				uCandidates[i] = uini + (incrU*(float)i/(float)subdivionParts);
				trianglesToTest[i] = triangleSequence[idxIni];
			}

			dCandidates[0] = distances[idxIni];
			dCandidates[dCandidates.size()-1] = distances[idxFin];
		}
		else
		{
			idxIni = minDistId-1;
			int idxCenter = minDistId;
			idxFin = minDistId+1;

			uini = uParameter[idxIni];
			ufin = uParameter[idxCenter];
			
			vini = refPoints[idxIni];
			vfin = refPoints[idxCenter];

			candidates.resize(subdivionParts*2+1);
			uCandidates.resize(subdivionParts*2+1);
			dCandidates.resize(subdivionParts*2+1);
			trianglesToTest.resize(subdivionParts*2+1);

			Vector3d dir = vfin-vini;
			float incrU = ufin-uini;

			for(int i = 0; i<= subdivionParts; i++)
			{
				candidates[i] = vini + (dir*(float)i/(float)subdivionParts);
				uCandidates[i] = uini + (incrU*(float)i/(float)subdivionParts);
				trianglesToTest[i] = triangleSequence[idxCenter];
			}

			uini = uParameter[idxCenter];
			ufin = uParameter[idxFin];
			
			vini = refPoints[idxCenter];
			vfin = refPoints[idxFin];

			dir = vfin-vini;
			incrU = ufin-uini;

			for(int i = 1; i<= subdivionParts; i++)
			{
				candidates[i+subdivionParts] = vini + (dir*(float)i/(float)subdivionParts);
				uCandidates[i+subdivionParts] = uini + (incrU*(float)i/(float)subdivionParts);
				trianglesToTest[i+subdivionParts] = triangleSequence[idxFin];
			}

			dCandidates[0] = distances[idxIni];
			dCandidates[subdivionParts] = distances[idxCenter];
			dCandidates[dCandidates.size()-1] = distances[idxFin];
		}*/

	}

	for(int i = 0; i< candidates.size(); i++)
	{
		Vector3d& pt1 = triangleGroup[trianglesToTest[i]]->verts[0]->position;
		Vector3d& pt2 = triangleGroup[trianglesToTest[i]]->verts[1]->position;
		Vector3d& pt3 = triangleGroup[trianglesToTest[i]]->verts[2]->position;

		Vector3d coords;
		get3DBarCoords(pt1, pt2, pt3, candidates[i], coords);

		double d01x = BihDistances.get(triangleGroup[trianglesToTest[i]]->verts[0]->id, vertId);
		double d01y = BihDistances.get(triangleGroup[trianglesToTest[i]]->verts[1]->id, vertId);
		double d01z = BihDistances.get(triangleGroup[trianglesToTest[i]]->verts[2]->id, vertId);

		double d01 = d01x*coords.x() + d01y*coords.y() + d01z*coords.z();

		dCandidates[i] = d01;
	}

	double newMinDist = 999;
	int newMinDistIdx = -1;
	for(int i = 0; i< candidates.size(); i++)
	{
		if(newMinDistIdx < 0 || newMinDist > dCandidates[i])
		{
			newMinDist = dCandidates[i];
			newMinDistIdx = i;
		}
	}

	newDist = newMinDist;
	newUParam = uCandidates[newMinDistIdx];

	if(newDist > minDistance001)
		printf("Houston, tenemos un problema con el refinamiento\n");


}


void dicotomicSearch(int vertId, int minDistId, float minDistance001, vector<float>& distances, vector<float>& uParameter, vector<Vector3d>& refPoints, vector<GraphNodePolygon*>& triangleGroup, vector<int>& triangleSequence, symMatrix& BihDistances, float& newDist, float& newUParam)
{
	Vector3d pt0 , pt1, pt2;

	// Casos particulares.
	if(minDistId == 0)
	{
		newDist = minDistance001;
		newUParam = uParameter[minDistId];
		return; // Es la primera... tendremos que tratarlo bien luego.
	}
	if(minDistId == refPoints.size()-1)	
	{
		newDist = minDistance001;
		newUParam = uParameter[minDistId];
		return; // Es la primera... tendremos que tratarlo bien luego.
	}

	float d0 = distances[minDistId-1];
	float d2 = distances[minDistId+1];

	// Esta por en medio.

	pt0 = refPoints[minDistId-1];
	pt1 = refPoints[minDistId];
	pt2= refPoints[minDistId+1];

	Vector3d v01 = pt1-pt0;
	Vector3d v12 = pt2-pt1;
	Vector3d v02 = pt2-pt0;

	Vector3d pt01 = pt0 + (v01*0.1);
	Vector3d pt12 = pt2 - (v12*0.1);

	v01.normalize();
	v12.normalize();
	v02.normalize();

	int t0 = triangleSequence[minDistId-1];
	int t1 = triangleSequence[minDistId];
	int t2 = triangleSequence[minDistId];

	Vector3d coords01, coords12;
	get3DBarCoords(triangleGroup[t0]->verts[0]->position, triangleGroup[t0]->verts[1]->position, triangleGroup[t0]->verts[2]->position, pt01, coords01);
	get3DBarCoords(triangleGroup[t2]->verts[0]->position, triangleGroup[t2]->verts[1]->position, triangleGroup[t2]->verts[2]->position, pt12, coords12);

	double d01x = BihDistances.get(triangleGroup[t0]->verts[0]->id, vertId);
	double d01y = BihDistances.get(triangleGroup[t0]->verts[1]->id, vertId);
	double d01z = BihDistances.get(triangleGroup[t0]->verts[2]->id, vertId);

	double d01 = d01x*coords01.x() + d01y*coords01.y() + d01z*coords01.z();

	double d12x = BihDistances.get(triangleGroup[t2]->verts[0]->id, vertId);
	double d12y = BihDistances.get(triangleGroup[t2]->verts[1]->id, vertId);
	double d12z = BihDistances.get(triangleGroup[t2]->verts[2]->id, vertId);

	double d12= d12x*coords12.x() + d12y*coords12.y() + d12z*coords12.z();


	Vector2d res =  Vector2d((pt2-pt0).dot(v02),d2);

	Vector2d l1 = Vector2d((pt01-pt0).dot(v02), (d01-d0));
	Vector2d l2 = Vector2d((pt2-pt12).dot(v02), (d12-d2));

	getIntersecction2D(Vector2d(0,d0), l1.normalized(), res, l2.normalized());

	newDist = res.y();
	newUParam = uParameter[minDistId-1] + (res.x()/(pt2-pt0).dot(v02))*(uParameter[minDistId+1]-uParameter[minDistId-1]);
}


void BulgeDeformer::applyDeformation(Geometry* m, Geometry* mOrig, binding* b, AirRig* rig)
{
	// Solo tenemos dos pointers, current y el padre... quizás podría eliminarse este
	// informacion repetida.

	DefGroup *dg = NULL, *dgParent = NULL;
	
	// El primer grupo corresponde al padre.
	int currentDGFatherIdx = groups[0]->childDeformerId;
	if(currentDGFatherIdx >= 0)
		dgParent = rig->defRig.defGroupsRef[currentDGFatherIdx];

	if(!dgParent)
	{
		printf("Hay algun problema, no tenemos padre...y todavia no se procesar esto.\n"); fflush(0);
		assert(false);
	}
	else
	{
		// Aqui procesariamos al padre, pero de momento lo saltamos
		// debería cogerse una media de los hijos para la dirección de arrugas,
		// dependiendo de cuales estén abiertos... no? Aunque no esta del todo
		// claro.
	}

	// Al menos tiene un hijo
	int currentDGIdx = groups[1]->deformerId;
	dg = rig->defRig.defGroupsRef[currentDGIdx];
	
	// Tenemos que asegurar que el indice principal existe
	if(currentDGIdx < 0 ) return;

	// Solo calculamos hacia los hijos, y del hijo hacia el padre.
	for(int bgIdx = 1; bgIdx < groups.size(); bgIdx++)
	{
		// Cogemos el hijo
		int childDGIdx = groups[bgIdx]->childDeformerId;
		DefGroup* dgChild = rig->defRig.defGroupsRef[childDGIdx];

		if(dgChild == NULL) continue;

		groups[bgIdx]->pressure.clear();
		groups[bgIdx]->pressurePoints.clear();

		// Obtenemos desplazamientos.
		Vector3d v1 = dgParent->getTranslation(false) - dg->getTranslation(false);
		Vector3d v2 = dgChild->getTranslation(false) - dg->getTranslation(false);

		float childBoneLength = v2.norm();

		v1.normalize();
		v2.normalize();

		// Esto controla si estan alineados -> en ese caso no aplica ningun efecto
		if(fabs(fabs(v1.dot(v2))-1) < SMALL_NUM) return;

		Vector3d v3 = dgChild->getTranslation(false) - dgParent->getTranslation(false);
		Quaterniond q; q.setFromTwoVectors(v1,v2);
		Vector3d bisectriz = q.slerp(0.5, Quaterniond::Identity())._transformVector(v1);
		bisectriz.normalize();

		// La IK debería haber colocado bien los joints... aqui partimos de que esta todo bien alineado.
		// Obtenemos los planos centrados en dg
		Vector3d nT = v1.cross(v2).normalized();
		Vector3d nC = nT.cross(bisectriz).normalized();
		Vector3d nB = nC.cross(nT).normalized();

		Vector3d planeOrigin = dg->getTranslation(false);

		// inicializacion para construir la curva antes de tocar nada.
		vector<GraphNodePolygon*> triangleGroup;
		vector<GraphNodePolygon*> origTriangleGroup;
		vector< vector<int> > triVertMap;
		triVertMap.resize(m->nodes.size());

		for(int triIdx = 0; triIdx < m->triangles.size(); triIdx++)
			for(int triVertIdx = 0; triVertIdx < 3; triVertIdx++)
				triVertMap[m->triangles[triIdx]->verts[triVertIdx]->id].push_back(triIdx);

		vector< bool > triPushed;
		triPushed.resize(m->triangles.size(), false);

		for(int vertIdx = 0; vertIdx < groups[bgIdx]->vertexGroup.size(); vertIdx++)
		{
			for(int vertTriIdx = 0; vertTriIdx < triVertMap[groups[bgIdx]->vertexGroup[vertIdx]].size(); vertTriIdx++)
			{
				int idx = triVertMap[groups[bgIdx]->vertexGroup[vertIdx]][vertTriIdx];
				if(!triPushed[idx])
				{
					triangleGroup.push_back(m->triangles[idx]);
					origTriangleGroup.push_back(mOrig->triangles[idx]);
					triPushed[idx] = true;
				}
			}
		}

		int firstTri = -1, lastTri = -1;

		// Get first curve Point
		Vector3d curveOrigin(0,0,0);
		Vector3d D(0,0,0);
		int typeO = INT_NO_INT;

		// Obtenemos el rayo bisectriz con el esqueleto en reposo.
		Vector3d planeOriginOriginalModel = dg->getRestTranslation(false);
		Vector3d bisectriz2 = dg->getRotation(false).inverse()._transformVector(bisectriz);
		Vector3d bisectrizOriginalModel = dg->getRestRotation(false)._transformVector(bisectriz2);
		Vector3d coordsProjectionOriginalModel;

		// Obtenemos la interseccion del rayo con el modelo en reposo.
		firstTri = getRayProjectionCoords(planeOriginOriginalModel, bisectrizOriginalModel, mOrig, origTriangleGroup, coordsProjectionOriginalModel, typeO);
		 
		//Vector3d otherCurveOrigin;
		//int firstTri002;
		//firstTri002 = getRayProjection(planeOrigin, bisectriz, m, triangleGroup, otherCurveOrigin, typeO);

		// Calculo del punto deformado, segun las coordenadas.
		Vector3d pt0 = triangleGroup[firstTri]->verts[0]->position;
		Vector3d pt1 = triangleGroup[firstTri]->verts[1]->position;
		Vector3d pt2 = triangleGroup[firstTri]->verts[2]->position; 
		curveOrigin = pt0*coordsProjectionOriginalModel.x()+
					  pt1*coordsProjectionOriginalModel.y()+
					  pt2*coordsProjectionOriginalModel.z();

		// TODEBUG:  QUIZAS SERÍA NECESARIO CALCULAR TAMBIÉN EL DESTINO DE LA MISMA MANERA.

		// Get final curve Point
		float percent = 1;
		int typeD = INT_NO_INT;
		Vector3d endBone = dg->getTranslation(false) + (dgChild->getTranslation(false) - dg->getTranslation(false))*percent;
		Vector3d nBone = -(nT.cross(v2));
		lastTri = getRayProjection(endBone, nBone, m, triangleGroup, D, typeD);

		// Procedemos al corte... hay algunos saltos... hay que ver de que va...
		// Proyectar los puntos que toca
		vector<Vector3d> displacements(groups[bgIdx]->vertexGroup.size());
		vector<bool> displaced(groups[bgIdx]->vertexGroup.size());
		float acumPressureChild = 0;
		float acumPressureFather = 0;

		float maxPressureFather = 0;
		float maxPressureChild = 0;

		Vector2f minI(99,99), maxI(-99,-99);

		vector<bool> toModifyWithBulge;
		toModifyWithBulge.resize(groups[bgIdx]->vertexGroup.size(), false);

		for(int k = 0; k < groups[bgIdx]->vertexGroup.size(); k++)
		{
			int idxPt = groups[bgIdx]->vertexGroup[k];
			// Proyectar el punto y guardarme el desplazamiento

			PointData* pd = &(b->pointData[idxPt]);
			Vector3d dir = (pd->node->position-planeOrigin);

			float a = dir.dot(nC);

			DefNode* df = rig->defRig.defNodesRef[pd->segmentId];

			if(a > 0 && df->boneId == currentDGFatherIdx)
			{
				// Esta asignado al padre y está en terreno del hijo
				displaced[k] = true;

				Vector3d I = -(a)*nC+ pd->node->position;
				
				// De momento solo proyectamos, por eso esta comentado todo esto
				/*
					displacements[k] = pd->node->position - I;
					float pressure = displacements[k].norm();
					maxPressureFather = max(maxPressureFather, pressure);
					acumPressureFather+= pressure;
				*/
				
				// proyeccion talcu
				pd->node->position = I;
				
			}
			
			if(df->boneId == currentDGIdx)
			{
				if(a < 0)
				{
					// Esta asignado al hijo y está en terreno del padre.
					displaced[k] = true;
					Vector3d I = -(a)*nC+ pd->node->position;
					displacements[k] = pd->node->position - I;

					// Pressure computation
					float pressure = displacements[k].norm();
					maxPressureChild = max(maxPressureChild, pressure);
					acumPressureChild+= pressure;

					// Loading pressure measuremments of this point over the plane
					Vector2f pressPoint(nT.dot(I-planeOrigin), nB.dot(I-planeOrigin));
					groups[bgIdx]->pressurePoints.push_back(pressPoint);
					groups[bgIdx]->pressure.push_back(pressure);

					minI[0] = min(minI[0], pressPoint[0]);
					minI[1] = min(minI[1], pressPoint[1]);

					maxI[0] = max(maxI[0], pressPoint[0]);
					maxI[1] = max(maxI[1], pressPoint[1]);

					// proyeccion talq
					pd->node->position = I;

					// Prueba... vamos a modificar todos... a ver
					toModifyWithBulge[k] = true;
				}
				else
					toModifyWithBulge[k] = true;
			}
			else displaced[k] = false;
		}

		// El origen y destino ya estan calculados.
		groups[bgIdx]->defCurve->origin = curveOrigin;
		groups[bgIdx]->defCurve->D = D;

		assert(firstTri >= 0 && lastTri >= 0);

		// Build curve
		groups[bgIdx]->refPoints.clear();
		groups[bgIdx]->refCoords.clear();
		groups[bgIdx]->triangleSequence.clear();

		getCurvePoints(curveOrigin, typeO, D, nBone.normalized(), v2.normalized(), triangleGroup, triVertMap, 
			groups[bgIdx]->refPoints, groups[bgIdx]->refCoords,  groups[bgIdx]->triangleSequence, firstTri, lastTri);

		groups[bgIdx]->minI = minI;
		groups[bgIdx]->maxI = maxI;

		// Compute curve height.
		Vector3d Length = dgChild->getTranslation(false) - dg->getTranslation(false);
		float ND =  (Length.norm()-maxPressureChild)/2;
		float h = sqrt((maxPressureChild*maxPressureChild)/4 + ND*maxPressureChild);

		//groups[bgIdx]->defCurve->W.clear();
		//groups[bgIdx]->defCurve->W.addPoint(0.0, 0.0);
		//groups[bgIdx]->defCurve->W.addPoint(0.25, maxPressureChild/Length.norm()*1.5);
		//groups[bgIdx]->defCurve->W.addPoint(1.0, 0.0);

		//groups[bgIdx]->defCurve->R.clear();
		//groups[bgIdx]->defCurve->R.addPoint(0.0, 0.0);
		//groups[bgIdx]->defCurve->R.addPoint(0.5, 0.0);
		//groups[bgIdx]->defCurve->R.addPoint(1.0, 0.0);

		// TO CHANGE -> new curve parametrization
		Vector3d newOrigin = curveOrigin;
		Vector3d newDir = (D-newOrigin).normalized();

		Vector3d nCurve = -(nT.cross(newDir.normalized()));

		groups[bgIdx]->defCurve->init(newDir, nCurve, groups[bgIdx]->refPoints, groups[bgIdx]->refCoords, maxPressureChild);

		vector<float> arcLenghtPoses(groups[bgIdx]->refPoints.size());
		vector<float> uParameter(groups[bgIdx]->refPoints.size());
		float currentLength = 0;
		for(int i = 0; i< groups[bgIdx]->refPoints.size(); i++)
		{
			if(i > 0)
			{
				currentLength += (groups[bgIdx]->refPoints[i]-groups[bgIdx]->refPoints[i-1]).norm();
			}

			arcLenghtPoses[i] = currentLength;
		}
		for(int i = 0; i< groups[bgIdx]->refPoints.size(); i++)
		{
			uParameter[i] = arcLenghtPoses[i]/currentLength;
		}

		float length = (D-newOrigin).norm(); 

		groups[bgIdx]->defCurve->r = 1;
		groups[bgIdx]->defCurve->lenght = length*percent;
		groups[bgIdx]->defCurve->high = length;
		//groups[bgIdx]->w.clear();
		//groups[bgIdx]->u.clear();

		// Perform the actual deformation
		for(int k = 0; k < toModifyWithBulge.size(); k++)
		{
			if(!toModifyWithBulge[k]) continue;

			if(groups[bgIdx]->refPoints.size() <= 3) continue;

			int idxPt = groups[bgIdx]->vertexGroup[k];
			// Proyectar el punto y guardarme el desplazamiento

			PointData* pd = &(b->pointData[idxPt]);
			Vector3d dir = (pd->node->position-newOrigin);
			float a = dir.dot(nC);

			DefNode* df = rig->defRig.defNodesRef[pd->segmentId];

			vector<float> distances;
			distances.resize(groups[bgIdx]->refPoints.size());
			
			//Projection over rest curve
			// First: linear search over the curve
			float minDistance001 = 9999;
			int minDistId = -1;
			for(int i = 0; i< groups[bgIdx]->refPoints.size(); i++)
			{
				int ptIdx0 = triangleGroup[groups[bgIdx]->triangleSequence[i]]->verts[0]->id;
				int ptIdx1 = triangleGroup[groups[bgIdx]->triangleSequence[i]]->verts[1]->id;
				int ptIdx2 = triangleGroup[groups[bgIdx]->triangleSequence[i]]->verts[2]->id; 

				int tri = groups[bgIdx]->triangleSequence[i];

				// WARNING: ONE A MATRIX
				assert(ONLY_ONE_A_MATRIX);
				// Usamos la primera matriz... deberiamos llevar un control de esto.
				double d1 = b->BihDistances[0].get((int)pd->node->id,ptIdx0);
				double d2 = b->BihDistances[0].get((int)pd->node->id,ptIdx1);
				double d3 = b->BihDistances[0].get((int)pd->node->id,ptIdx2);

				double dist = groups[bgIdx]->refCoords[i].x()*d1 + 
							  groups[bgIdx]->refCoords[i].y()*d2 + 
							  groups[bgIdx]->refCoords[i].z()*d3; 

				distances[i] = dist;

				if(minDistId < 0 || minDistance001 > dist)
				{
					minDistId = i;
					minDistance001 = dist;
				}
			}

			float newDist, newParam;
			// Second: taking the pre and post points we refine the rearch to get a better result.
			//dicotomicSearch((int)pd->node->id, minDistId, minDistance001, distances, uParameter, groups[bgIdx]->refPoints, triangleGroup, 
			///				groups[bgIdx]->triangleSequence, b->BihDistances[0] , newDist, newParam );

			refineParameter((int)pd->node->id, minDistId, minDistance001, distances, uParameter, groups[bgIdx]->refPoints, triangleGroup, 
							groups[bgIdx]->triangleSequence, b->BihDistances[0] , newDist, newParam );

			Vector3d proj = newDir.dot(pd->node->position - newOrigin)*newDir;
			float val = newDir.dot(pd->node->position - newOrigin);
			float u = val/length;

			if(val < 0) u = 0;

			//float u = uParameter[minDistId];
			u = newParam;

			//Weight depending on the distance
			//float dist = (proj+newOrigin-pd->node->position).norm();
			//float w = groups[bgIdx]->defCurve->implicitFunction(dist);
			//float w = groups[bgIdx]->defCurve->implicitFunction(minDistance001);
			float w = groups[bgIdx]->defCurve->implicitFunction(newDist);

			groups[bgIdx]->w[k] = w;
			groups[bgIdx]->u[k] = u;

			Vector3d displ(0,0,0);
			groups[bgIdx]->defCurve->getDisplacement(pd->node->position, u, w, displ);

			pd->node->position += displ;

		}

		//printf("h: %f sobre %f, ND:%f.\n", h, Length.norm(), ND);
		//printf("Max. Preasure: father->%f, child->%f.\n", maxPressureFather, maxPressureChild);
		//printf("Preasure Child:%f Father:%f\n", acumPressureFather, acumPressureChild);
	}
}