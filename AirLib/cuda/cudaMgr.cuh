#ifndef CUDA_MGR_H
#define CUDA_MGR_H

#include "cudaDefinitions.cuh"

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

#ifndef __CUDACC__ 
	#define __CUDACC__
#endif

#include <cuda_texture_types.h>
#include <texture_types.h>
#include <texture_indirect_functions.h>

#include <vector_types.h>

#include "lock.cuh"

class cudaNode
{
public:

	cudaNode() 
	{
		cudaWeights = NULL;
		cudaIndirection = NULL;
		cudaDistances = NULL;
		indSize = 0;
		precomputedDist = 0; 
		nodeId = -1; 
	}

	cudaNode(int npts) 
	{
		checkCudaErrors( cudaMalloc( (void**)&cudaWeights, npts * sizeof(PRECISION) ) );
		cudaError_t cudaStatus = cudaGetLastError();
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "reservar memoria para pesos : %s\n", cudaGetErrorString(cudaStatus));
			fflush(0);
			return;
		}
		
		/*
		checkCudaErrors( cudaMalloc( (void**)&cudaDistances, npts * sizeof(PRECISION) ) );
		cudaStatus = cudaGetLastError();
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "reservar memoria para distancias : %s\n", cudaGetErrorString(cudaStatus));
			fflush(0);
			return;
		}

		checkCudaErrors( cudaMalloc( (void**)&cudaIndirection, npts * sizeof(int) ) );
		 cudaStatus = cudaGetLastError();
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "Reservar memoria para la indireccion : %s\n", cudaGetErrorString(cudaStatus));
			fflush(0);
			return;
		}
		*/

		indSize = 0;
		precomputedDist = 0; 
		nodeId = -1; 
	}

	~cudaNode()
	{
		cudaFree(cudaWeights);
		cudaFree(cudaIndirection);
		cudaFree(cudaDistances);

		cudaWeights = NULL;
		cudaIndirection = NULL;
		cudaDistances = NULL;
	}

	cudaNode(const cudaNode& in) 
	{
		nodeId = in.nodeId;
		cudaWeights = in.cudaWeights;
		cudaIndirection = in.cudaIndirection;
		indSize = in.indSize;
		cudaDistances = in.cudaDistances;
		precomputedDist = in.precomputedDist;
	}

	int nodeId; // (HOST)

	// MVC weights
	PRECISION* cudaWeights; // (DEVICE)
	int* cudaIndirection; // (DEVICE)
	int indSize; // (HOST)

	// Distances from this node to each other node
	PRECISION* cudaDistances; // (DEVICE)

	// Precomputed distance for faster results 
	PRECISION precomputedDist; // (HOST)
};


class cudaModel
{
public:
	cudaModel()
	{
		cudaNormas = NULL;
		cudaUnitVectors = NULL;
		cudaCoords = NULL;
		cudaWeights = NULL;

		hostPositions = NULL;
		hostTriangles = NULL;
		hostBHDistances = NULL;
		tempResults = NULL;

		ntri = npts = 0; 
		id = 0;
		pointsLoaded = 0;
	}

	cudaModel(int _id)
	{
		cudaNormas = NULL;
		cudaUnitVectors = NULL;
		cudaCoords = NULL;
		cudaWeights = NULL;

		hostPositions = NULL;
		hostTriangles = NULL;
		hostBHDistances = NULL;
		tempResults = NULL;

		ntri = npts = 0; 
		id = _id;

		pointsLoaded = 0;

	}

	cudaModel(const cudaModel& in) 
	{
		cudaNormas = in.cudaNormas;
		cudaUnitVectors = in.cudaUnitVectors;
		cudaCoords = in.cudaCoords;
		cudaWeights = in.cudaWeights;

		cudaSum = in.cudaSum;
		cudaUnitVectors = in.cudaUnitVectors;
		resWeights = in.resWeights;
		tempResults = in.tempResults;

		hostPositions = in.hostPositions;
		hostTriangles = in.hostTriangles;
		hostBHDistances = in.hostBHDistances;
		tempResults = in.tempResults;

		ntri = in.ntri;
		npts = in.npts; 

		id = in.id;

		cudaPositionsTex = in.cudaPositionsTex;
		cudaTrianglesTex = in.cudaTrianglesTex;
		cudaBHDistancesTex = in.cudaBHDistancesTex;
		cudaNormasTex = in.cudaNormasTex;
		cudaUnitVectorsTex = in.cudaUnitVectorsTex;

		cudaPositions_forTex = in.cudaPositions_forTex;
		cudaTriangles_forTex = in.cudaTriangles_forTex;
		cudaBHDistances_forTex = in.cudaBHDistances_forTex;

		for(int i = 0; i< nodes.size(); i++)
			nodes[i] = in.nodes[i];

		pointsLoaded = in.pointsLoaded;

		newNode = in.newNode;
	}

	int id;

	// Host side
	unsigned int npts;
	unsigned int ntri;

	PRECISION* hostPositions;
	int* hostTriangles;
	float* hostBHDistances;
	
	int* lockVertexes;
	int* singleLock;

	int pointsLoaded;
	thrust::host_vector<cudaNode*> nodes;

	// Device side
	// Gloabl texture objects
	cudaTextureObject_t cudaPositionsTex;
	cudaTextureObject_t cudaTrianglesTex;
	cudaTextureObject_t cudaBHDistancesTex;

	// Temporal texture object
	cudaTextureObject_t cudaNormasTex;
	cudaTextureObject_t cudaUnitVectorsTex;

	cudaNode newNode;

	float* cudaPositions_forTex;
	int* cudaTriangles_forTex;
	float* cudaBHDistances_forTex;

	// Temp for faster computations
	PRECISION* cudaSum;

	PRECISION* cudaNormas;
	PRECISION3* cudaUnitVectors;

	PRECISION* resWeights;
	PRECISION* cudaWeights;

	PRECISION* cudaCoords;
	PRECISION* tempResults;
};

class cudaManager
{
public:

	cudaManager()
	{
		loaded = false;
		models = NULL;

		modelsLoaded = 0;
	}

	cudaModel* models;
	int modelsLoaded;

	bool loaded;
	int blocksPerGridPoints;
	int blocksPerGridFaces;

	// Allocate and load data
	void loadModels(cudaModel* in_models, int modelsCount, bool collectData = true);
	void freeModels(bool collectData = true);

	// Compute MVC coords
	void cudaMVC(PRECISION3& point, 
				 PRECISION* weights,
				 int modelId,
				 int pointId,
				 bool collectData = true);

	// Compute precomputed distance for one node
	double Precompute_Distances_CUDA(double* weights,
									int indSize,
									int* indirection,
									int modelIdx,
									int nodeIdx);

};

void mvcAllBindings(PRECISION3 point, PRECISION* weights, cudaModel* modelo);

int mainCuda();

void getPrecomputedDistance(int node);
void getDistances(int node,int vert);

#endif