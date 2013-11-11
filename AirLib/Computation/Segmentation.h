#ifndef VORONOICOMP_H
#define VORONOICOMP_H

#include "DataStructures/grid3D.h"
#include "DataStructures/SurfaceData.h"
#include "DataStructures/DataStructures.h"

#define DOUBLESIZE 8
#define MBSIZEINBYTES 1048576
#define FIRST_ITERATION -1

#define useMVC true

#include <utils/util.h>

using namespace std;

class skeleton;
class gridRenderer;
class Modelo;
class scene;
class joint;
class sceneUpdatingFlags;

class embedVector
{
public:
	embedVector(){}

	double norm;
	vector<double> unitVector;
	vector<double> embeding;
};

/*
void computeSkinning(MyMesh& mesh,
                     grid3d& grid,
                     voronoiGraphData& v,
                     vector< vector<double> >& embeddedPoints,
                     vector< vector<double> >& embedding,
                     bool onlyBorders = false);
*/

void buildVoronoiGraph(grid3d& grid,
                       vector< vector<bool> >& voronoiGraph,
                       int graphSize,
                       bool onlyBorders = false);

int weightsComputationEmbedded(grid3d& grid,
                               vector< vector<double> >& points,
                               vector< vector<bool> >& voronoiGraph);

int segmentVolumeWithEmbedding(Modelo& m, binding *bd);

int segmentVolumeWithEmbeddingPropagation( grid3d& grid,
                                vector< DefNode >& points,
                                vector< vector<double> >& embeddedPoints);

int segmentVolume( grid3d& grid,
                   vector<DefNode> &points);

// Segment volume with labels constructing a voronoi diagram
int PropagateWeights(grid3d &grid,
                     vector<DefNode> &points,
                     vector<vector<bool> > &voronoiGraph,
                     bool onlyBorders = false);

int PropagateWeightsFromSkt(grid3d& grid,
                     skeleton *skt,
                     float smoothPropagationRatio,
                     bool onlyBorders = false);

int PropagateWeightsFromJoint(grid3d& grid,
                              int fatherNodeId,
                              int boneId,
                              float smoothPropagationRatio, bool onlyBorders);

void traduceSegmentation(grid3d& grid, vector<int>& traductionTable);

void traducePartialSegmentation(Modelo &grid, map<int, int> &traductionTable);

/*
void gridInit(MyMesh& mesh,
               grid3d& grid);

int gridCellsEmbeddingInterpolation(MyMesh& mesh,
                                    grid3d& grid,
                                    vector< vector<double> >& embedding,
                                    bool onlyBorders);
*/

void initGridFlagsForFrontPropagation(grid3d& grid, int idFront);
void initfrontWithBoundaries(grid3d& grid, vector< Vector3i >& front, int nodeId);

void borderFrontSmoothing(grid3d& grid,
						  vector< Vector3i >& front,
						  vector<Vector3i>& postProcess,
						  float smoothPropagationRatio,
						  int idFront);

void createTraductionTable(joint* jt, std::map<int, int>& traductionTable, int idNode, bool bRoot = false);
void initDomainForId(grid3d& grid, int fatherId);
void PropagateFromSegment(Modelo &m, binding *bd, int frontId);
void computeHierarchicalSkinning(Modelo &m, binding *bb);
void computeHierarchicalSkinning(Modelo &m, binding *bb, joint* jt);

void initDomain(Modelo& m, binding* bd, int domainId_init);
void updateSkinningWithHierarchy(Modelo &m);
void updateSkinning(Modelo &m);

//void ComputeSkining(Modelo* m, grid3d& grid);

void ComputeSkining(Modelo &m);

void Voxelize(scene* snc, Modelo* m, float resolution, float worldScale, bool onlyBorders = true);
//void computeSecondaryWeights(Modelo* m);

void PrecomputeDistances(vector< vector<double> >& embeddedPoints, vector< vector<int> >& weightsIndirection, symMatrix& BihDistances, vector< DefNode >& intPoints);
double PrecomputeDistancesSingular(vector<double>& weights, symMatrix& BihDistances);
double PrecomputeDistancesSingular_sorted(vector<double>& weights, vector<int>& indirection, symMatrix& BihDistances, double threshold);
double PrecomputeDistancesSingular_sorted(symMatrix& BihDistances, vector<weight>& weights);

double PrecomputeDistancesSingular_block(vector<double>& weights, symMatrix& BihDistances, int iniIdx, int finIdx);
void doubleArrangeElements(vector<double>& weights, vector<int>& orderedIndirection, bool printStatistics, double& threshold);
void doubleArrangeElements_withStatistics(vector<double>& weights, vector<int>& orderedIndirection, vector<double>& statistics, double& threshold);

void doubleArrangeElements_wS_fast(vector<double>& weights, vector<int>& orderedIndirection, double& threshold);

void propagateIdFromNode(int id, vector<int>& frontIds,vector<int>& harvestIds,vector<bool>& visIds,vector<int>& connIds,vector<GraphNode*> nodes);
void renameLinks(vector<int>& links, int group, int newGroup);
int contains(vector<int>& values, int value);
void FusionBindings(Modelo& m, vector<vector<int> >& groupBindings);
void AddVirtualTriangles(Modelo& m);
void BuildSurfaceGraphs(Modelo& m, vector<binding*>& bindings);
void normalizeDistances(Modelo& m);
bool ComputeEmbeddingWithBD(Modelo& model, bool withPatches = false);
bool LoadEmbeddings(Modelo& m, char* bindingFileName);
void SaveEmbeddings(Modelo& model, char* fileName, bool ascii = false);


// For hierarchical skinning
void clearOlderComputations(Modelo& m);
void initDomain(Modelo& m, binding* bd, int domainId_init);
void traducePartialSegmentation(Modelo& m, binding* bd, map<int, int>& traductionTable);
void SmoothFromSegment(Modelo& m, binding* bd, int frontId);
void initSurfaceWeightsSmoothing(Modelo& m, binding* bd, vector< int >& front, int nodeId);
void weightsSmoothing(Modelo& m, binding* bd,
                      vector< int >& front,
                      float smoothPropagationRatio,
                      int idFront,
					  int smoothingPasses);

#endif // VORONOICOMP_H
