#ifndef MV_INTERIORDISTANCES_H
#define MV_INTERIORDISTANCES_H

#include <vector>
#include <Eigen/Geometry>

#include "DataStructures/DataStructures.h"
#include "DataStructures/SurfaceData.h"
#include "DataStructures/Modelo.h"
#include <time.h>

#include <math.h>

#include "utils/util.h"

using namespace std;

//void mvcEmbedPoint( Vector3d& point, vector<double>& embeddedPoint, vector< vector<double> >& V, MyMesh& modelo);
//void mvc_embedding(vector< Vector3d>& points, vector< vector<double> >& embeddedPoints,vector< vector<double> >& V, MyMesh& modelo);

void mvc_weights(vector< Vector3d>& auxPoints, vector< vector<double> >& embeddedPoints, binding* bd, Modelo& m);
void mvcSingleBinding( Vector3d& point, vector<double>& embeddedPoint, binding* bd, Modelo& modelo);
void mvcAllBindings( Vector3d& point, vector<double>& embeddedPoint, Modelo& modelo);
void mvcOpt(Vector3d& point, MatrixXf& weights, int id, Modelo& modelo);
#endif // UTILGL_H
