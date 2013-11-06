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

void mvcEmbedPoint(vcg::Point3d& point, vector<double>& embeddedPoint, vector< vector<double> >& V, MyMesh& modelo);
void mvc_embedding(vector<vcg::Point3d>& points, vector< vector<double> >& embeddedPoints,vector< vector<double> >& V, MyMesh& modelo);

void mvc_weights(vector<vcg::Point3d>& auxPoints, vector< vector<double> >& embeddedPoints, binding* bd, Modelo& m);
void mvcSingleBinding(vcg::Point3d& point, vector<double>& embeddedPoint, binding* bd, Modelo& modelo);
void mvcAllBindings(vcg::Point3d& point, vector<double>& embeddedPoint, Modelo& modelo);
#endif // UTILGL_H
