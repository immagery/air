#ifndef IOWEIGHTS_H
#define IOWEIGHTS_H

#include "..\DataStructures\grid3d.h"
#include "..\DataStructures\Geometry.h"

void saveWeightsToMaya(Geometry* mesh,
                       vector< vector<float> >& meshWeights,
                       vector<string>& names,
                       vector<Point3d>& points,
                       string sFile);

void saveWeightsToMayaByName(Geometry* mesh,
                             vector< vector<float> >& meshWeights,
                             vector<string>& names,
                             string sFile);


#endif // IOWEIGHTS_H
