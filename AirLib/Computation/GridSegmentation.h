#ifndef VORONOICOMP_H
#define VORONOICOMP_H

#include "..\DataStructures\grid3D.h"
#include "..\DataStructures\DataStructures.h"

#define DOUBLESIZE 8
#define MBSIZEINBYTES 1048576

#include <vector>
#include <map>

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

void computeSkinning(MyMesh& mesh,
                     grid3d& grid,
                     voronoiGraphData& v,
                     vector< vector<double> >& embeddedPoints,
                     vector< vector<double> >& embedding,
                     bool onlyBorders = false);

void buildVoronoiGraph(grid3d& grid,
                       vector< vector<bool> >& voronoiGraph,
                       int graphSize,
                       bool onlyBorders = false);

int weightsComputationEmbedded(grid3d& grid,
                               vector< vector<double> >& points,
                               vector< vector<bool> >& voronoiGraph);

int segmentVolumeWithEmbedding(grid3d &grid,
                               vector< DefNode > &points,
                               vector<vector<double> > &embeddedPoints,
                               bool onlyBorders = false);

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

void traducePartialSegmentation(grid3d& grid, map<int, int> &traductionTable);

void gridInit(MyMesh& mesh,
               grid3d& grid);

int gridCellsEmbeddingInterpolation(MyMesh& mesh,
                                    grid3d& grid,
                                    vector< vector<double> >& embedding,
                                    bool onlyBorders);

void initGridFlagsForFrontPropagation(grid3d& grid, int idFront);
void initfrontWithBoundaries(grid3d& grid, vector< Point3i >& front, int nodeId);

void borderFrontSmoothing(grid3d& grid,
						  vector< Point3i >& front,
						  vector<Point3i>& postProcess,
						  float smoothPropagationRatio,
						  int idFront);

void createTraductionTable(joint* jt, std::map<int, int>& traductionTable, int idNode, bool bRoot = false);
void initDomainForId(grid3d& grid, int fatherId);
void PropagateFromSegment(grid3d& grid, int frontId);
void computeHierarchicalSkinning(grid3d& grid);
void computeHierarchicalSkinning(grid3d& grid, joint* jt);

void initGridForHierarchicalskinning(grid3d& grid, int domainId_init);
void updateSkinningWithHierarchy(grid3d& grid);
void updateSkinning(grid3d& grid);
void ComputeSkining(Modelo* m, grid3d& grid);
void ComputeSkining(Modelo* m, grid3d& grid, sceneUpdatingFlags* updatingInfo);

void Voxelize(scene* snc, Modelo* m, float resolution, float worldScale, bool onlyBorders = true);
void BindSkeletons(scene* snc, grid3d* grid);
void computeSecondaryWeights(Modelo* m);

void reportResults(Modelo* model);


#endif // VORONOICOMP_H
