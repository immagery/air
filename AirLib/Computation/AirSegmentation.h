#ifndef AIR_SEGMENTATION_H
#define AIR_SEGMENTATION_H

#define DOUBLESIZE 8
#define MBSIZEINBYTES 1048576
#define FIRST_ITERATION -1

#define useMVC true

#define VERBOSE_PROCESS true

#include <DataStructures\AirRig.h>

class ordWeight 
{

public:
  int idx;
  float weight;

  virtual bool operator() (ordWeight& i,ordWeight& j) { return (i.weight > j.weight); }

};

class ordIdx: public ordWeight
{
public:
	virtual bool operator() (ordWeight& i, ordWeight& j) { return (i.idx < j.idx); }
};

// Updates all the info for compute the skinning and computes it.
void updateAirSkinning(DefGraph& graph, Modelo& model);

// Segment the model using the air deformers.
void segmentModelFromDeformers(Modelo& model, binding* bd, DefGraph& graph);

void segmentModelFromDeformersOpt(  Modelo& model, binding* bd, DefGraph& graph, 
									MatrixXf& subDistances, 
									map<int, int>& matrixDefReference,
                                    MatrixXf& computedDistances,
									int computationSize);

// Compute the hierarchical skinning using computed segmentation
void propagateHierarchicalSkinning(Modelo& model, binding* bd);

void createTraductionTable(DefGroup* group, map<int, int>& traductionTable, int idNode, bool onlyRoot = false);
void createTraductionTableOpt(DefGroup* group, vector<int>& traductionTable, int idNode, bool onlyRoot = false);

void propagateHierarchicalSkinningOpt(Modelo& model, binding* bd, DefGraph& graph);
void propagateHierarchicalSkinningOpt(Modelo& model, binding* bd, DefGraph& graph, DefGroup& group, int& times, 
									  vector<float>& weightsT1, vector<float>& weightsT2, vector<unsigned int>& indicesToCompute);

void propagateHierarchicalSkinning(Modelo& model, binding* bd, DefGraph& graph);
void propagateHierarchicalSkinning(Modelo& model, binding* bd, DefGraph& graph, DefGroup& group);


// Compute secondary weights - now is not updating data, it computes what ever it needs.
void computeSecondaryWeights(Modelo& model, binding* bd, DefGraph& graph);

void computeSecondaryWeightsOpt(Modelo& model, binding* bd, 
                                DefGraph& graph, MatrixXf& subdistances, 
                                map<int, int>& idxOrder,
                                MatrixXf& computedDistances,
								bool wideValueComputation);

int indexOfNode(int nodeId, vector<DefNode>& nodes);

void clearOlderComputations(Modelo& m);

void initDomain(Modelo& m, binding* bd, int domainId_init);
void initDomainOpt(Modelo& m, binding* bd, int domainId_init);

bool LoadEmbeddings(Modelo& m, char* bindingFileName);

bool ComputeEmbeddingWithBD(Modelo& model, bool withPatches= false);

void doubleArrangeElements_wS_fast(vector<double>& weights, vector<int>& orderedIndirection, double& threshold);

int getSignificantWeights(vector<ordWeight>& weights, VectorXf& cuttedWeights, VectorXi& cuttedIndexes);

double PrecomputeDistancesSingular_sorted(vector<double>& weights, vector<int>& indirection, symMatrixLight& BihDistances, double threshold);

void initSurfaceWeightsSmoothing(Modelo& m, binding* bd, vector< int >& front, int nodeId);
void initSurfaceWeightsSmoothingOpt(Modelo& m, 
									binding* bd, 
									int nodeId, 
									vector<float>& weights,
									vector<float>& weightsAux,
									vector<unsigned int>& indicesToCompute,
									int& lastIndex);

void insertInThisPos(vector<double>& valuesOrdered, vector<int>& weightsIndirection, int element, int ff,double value,int& count);

void SaveEmbeddings(Modelo& model, char* fileName, bool ascii = false);

void traducePartialSegmentation(Modelo& m, binding* bd, map<int, int>& traductionTable);
void traducePartialSegmentationOpt(Modelo& m, binding* bd, vector<int>& traductionTable);

void weightsSmoothing(Modelo& m, binding* bd,
                      vector< int >& front,
                      float smoothPropagationRatio,
                      int idFront,
					  int smoothingPasses);

void weightsSmoothing_opt(Modelo& m, binding* bd,
						  float smoothPropagationRatio,
						  int idFront,
						  int smoothingPasses,
						  vector<float>& ptWT1,
						  vector<float>& ptWT2,
						  vector<unsigned int>& indicesToCompute,
						  int& lastIndex);

void initData(Modelo& m, binding* bd,
                      vector< int >& front,
                      float smoothPropagationRatio,
                      int idFront,
					  int smoothingPasses,
					  vector<float>& ptWT1,
					  vector<float>& ptWT2,
					  vector<bool>& visited,
					  vector< vector<int> >& neighbours,
					  vector<unsigned int>& indicesToCompute,
					  int& lastIndex);


// Compute the skinning only with dirty nodes.
void computeNodesOptimized(DefGraph& graph, Modelo& model, MatrixXf& MatrixWeights, MatrixXf& distancesTemp, map<int, int>& defNodeRef);

#endif // AIR_SEGMENTATION_H