#ifndef INTERIORDISTANCESDATA_H
#define INTERIORDISTANCESDATA_H

#include "DataStructures.h"
#include "SurfaceData.h"

using namespace std;

void WriteEmbedding(std::string fileName, vector< vector<double> >& V, bool ascii = true);
void ReadEmbedding(std::string fileName, vector< vector<double> >& V, bool ascii = true);

/*   ---- MEAN VALUE COORDINATES INTERPOLATION ---
  Interpolate points according to eigenVectors embedding
  previously calculated using MVC (Mean Value Coordinates)

  >  Points                -- list of points in 3D space to embed
  >  embeddedPoints        -- result
  >  eigenVectorsEmbedding -- embedding calculated with .... function
  >  modelo                -- model that constains the points.
*/
void mvc_IBD(vector<vcg::Point3d>& points,
             vector< vector<double> >& embeddedPoints,
             vector< vector<double> >& eigenVectorsEmbedding,
             MyMesh& modelo);

/*   ---- INTERIOR DISTANCE COMPUTATION ---
  Get the distance between two points embedded

  >  Source        -- point embedded (eigen vector)
  >  Target        -- point embedded (eigen vector)

*/
double distancesFromEbeddedPoints(vector<double>& source,
                                          vector<double>& target);

double distancesFromEbeddedPointsExtended(vector<double>& source,
                                          vector<double>& target,
                                          float ext = 1.0);

double BiharmonicDistanceP2P(vector<double>& weights, 
							int pointIdx, 
							binding* bd, 
							DefNode& node);

double BiharmonicDistanceP2P(vector<double>& weights, 
							int pointIdx, 
							binding* bd, 
							float ext, 
							float precomputedDistance);

double BiharmonicDistanceP2P_block(vector<double>& weights, 
									int pointIdx, 
									binding* bd, 
									float ext, 
									float precomputedDistance, 
									int iniIdx, 
									int finIdx);

double BiharmonicDistanceP2P_sorted(vector<double>& weights, 
									vector<int>& weightsSort, 
									int pointIdx, 
									binding* bd, 
									float ext, 
									float precomputedDistance,
									double threshold = 0.00001);


#endif // INTERIORDISTANCESDATA_H
