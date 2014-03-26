#ifndef AIR_SEGMENTATION_H
#define AIR_SEGMENTATION_H

#define DOUBLESIZE 8
#define MBSIZEINBYTES 1048576
#define FIRST_ITERATION -1

#define useMVC true

#include <DataStructures\AirRig.h>

// Updates all the info for compute the skinning and computes it.
void updateAirSkinning(DefGraph& graph, Modelo& model);

// Segment the model using the air deformers.
void segmentModelFromDeformers(Modelo& model, binding* bd, DefGraph& graph);

// Compute the hierarchical skinning using computed segmentation
void propagateHierarchicalSkinning(Modelo& model, binding* bd);

void createTraductionTable(DefGroup* group, map<int, int>& traductionTable, int idNode, bool onlyRoot = false);

void propagateHierarchicalSkinning(Modelo& model, binding* bd, DefGraph& graph);
void propagateHierarchicalSkinning(Modelo& model, binding* bd, DefGraph& graph, DefGroup& group);

// Compute secondary weights - now is not updating data, it computes what ever it needs.
void computeSecondaryWeights(Modelo& model, binding* bd, DefGraph& graph);

int indexOfNode(int nodeId, vector<DefNode>& nodes);

void clearOlderComputations(Modelo& m);

void initDomain(Modelo& m, binding* bd, int domainId_init);

bool LoadEmbeddings(Modelo& m, char* bindingFileName);

bool ComputeEmbeddingWithBD(Modelo& model, bool withPatches= false);

void doubleArrangeElements_wS_fast(vector<double>& weights, vector<int>& orderedIndirection, double& threshold);

double PrecomputeDistancesSingular_sorted(vector<double>& weights, vector<int>& indirection, symMatrixLight& BihDistances, double threshold);

void initSurfaceWeightsSmoothing(Modelo& m, binding* bd, vector< int >& front, int nodeId);

void insertInThisPos(vector<double>& valuesOrdered, vector<int>& weightsIndirection, int element, int ff,double value,int& count);

void SaveEmbeddings(Modelo& model, char* fileName, bool ascii = false);

void traducePartialSegmentation(Modelo& m, binding* bd, map<int, int>& traductionTable);

void weightsSmoothing(Modelo& m, binding* bd,
                      vector< int >& front,
                      float smoothPropagationRatio,
                      int idFront,
					  int smoothingPasses);

#endif // AIR_SEGMENTATION_H