#ifndef CUDA_MVC_H
#define CUDA_MVC_H

#include "cudaDefinitions.cuh"

#include "lock.cuh"

#define M_PI 3.14159265
#define tresh1 0.000001
#define tresh2 0.00001
#define tresh3 0.0001

PRECISION cudaSign(PRECISION v);
__device__ PRECISION cSign(PRECISION v);

__device__ PRECISION cDet(PRECISION3 u1, PRECISION3 u2, PRECISION3 u3);
__device__ PRECISION norm(PRECISION3& pt1);

__device__ PRECISION3 divide_Pt_f(PRECISION3& pt1, PRECISION val);
__device__ void divide_on_Pt_f(PRECISION3& pt1, PRECISION val);

__device__ PRECISION sq_norm(PRECISION3& pt1);

__global__ void unitVectorsComputation( int nv,
										PRECISION3 point,
										cudaTextureObject_t nodePos,
										//PRECISION3* nodePos,   	
										PRECISION3* unitVectors,  
										PRECISION* normas,
										PRECISION* weights,
										int* lockVertexes);

__global__ void coordsComputation( int nt, 
								   PRECISION3 point,
								   cudaTextureObject_t triangles,
								   cudaTextureObject_t unitVectors,
								   cudaTextureObject_t normas,
								   int* lockVertexes,
								   PRECISION* weights);

__global__ void sumWeightsRedux(int* lock,
								PRECISION* tempWeights, 
								int ntri, 
								PRECISION* cudaSum);

__global__ void sumWeightsParcial(int* lockVertexes,
								  PRECISION* tempWeights, 
								  int ntri, 
								  int npts, 
								  cudaTextureObject_t triangleIdx, 
								  PRECISION* weights);

__global__ void normalizeWeights(PRECISION* weights, PRECISION* sum, int nv);


/*
#define PRECISION double
#define PRECISION3 double3

#include <vector_types.h>

/*
using namespace std;
class cudaPoint3i
{
public:
	int x;
	int y;
	int z;

	int& operator[](const int in)
	{
		if(in == 0)
		{
			return x;
		}
		else if(in == 1)
		{
			return y;

		}
		else return z;
	}
};

class cudaPoint3d
{
public:
	double x;
	double y;
	double z;

	cudaPoint3d(){ x = 0; y = 0; z = 0;}
	cudaPoint3d(float3 in){ x = in.x; y = in.y; z = in.z;}
	cudaPoint3d(double _x, double _y, double _z){ x = _x; y = _y; z = _z;}

	cudaPoint3d operator+(const cudaPoint3d& in)
	{
		return cudaPoint3d(x+in.x, y+in.y, z+in.z);
	}

	cudaPoint3d operator/(const float in)
	{
		return cudaPoint3d(x/in, y/in, z/in);
	}

	cudaPoint3d operator-(const cudaPoint3d& in)
	{
		return cudaPoint3d(x-in.x, y-in.y, z-in.z);
	}

	double& operator[](const int in)
	{
		if(in == 0)
		{
			return x;
		}
		else if(in == 1)
		{
			return y;

		}
		else return z;
	}

	float dot(const cudaPoint3d& in)
	{
		return (x*in.x+ y*in.y+ z*in.z);
	}

	cudaPoint3d(const cudaPoint3d& in)
	{
		x = in.x;
		y = in.y;
		z = in.z;
	}

	float norm()
	{
		return sqrt(x*x+y*y+z*z);
	}

};

#define CUDA_NODE_SIZE sizeof(float3)+sizeof(int)
#define CUDA_TRIANGLE_SIZE sizeof(int)*3

struct cudaNode
{
public:
	float3 P;
	int Id;

	unsigned int getMemorySize()
	{
		return sizeof(float3)+sizeof(int);
	}
};


struct cudaTriangle
{
public:
	int3 verts;
	int v1;
	int v2;
	int v3;

	unsigned int getMemorySize()
	{
		return sizeof(int)*3;
	}
};

*/

	/*
class cudaModel
{
public:

	int id;

	PRECISION3* hostPositions;
	int3* hostTriangles;
	double* hostBHDistances;

	PRECISION3* cudaPositions;
	int3* cudaTriangles;
	double* cudaBHDistances;

	unsigned int npts;
	unsigned int ntri;

	// Temporal data for MVC
	PRECISION* cudaNormas;
	PRECISION3* cudaUnitVectors;

	PRECISION* resWeights;
	PRECISION* cudaWeights;

	PRECISION* cudaCoords;
	PRECISION* tempResults;

	static int getMemorySize()
	{
		// All the data are pointers or ints
		int size = sizeof(PRECISION3*)*3 + sizeof(PRECISION*)*5 + sizeof(unsigned int)*2 +
				   sizeof(int3*)*2 + sizeof(double*)*2 ;
		return size;
	}

	cudaModel(int _id)
	{
		cudaPositions = NULL;
		cudaTriangles = NULL;
		cudaNormas = NULL;
		cudaUnitVectors = NULL;
		cudaCoords = NULL;
		cudaWeights = NULL;
		cudaBHDistances = NULL;

		hostPositions = NULL;
		hostTriangles = NULL;
		hostBHDistances = NULL;
		tempResults = NULL;

		ntri = npts = 0; 

		id = _id;
	}

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
	int blocksPerGrid;

	// Allocate and load data
	void loadModels(cudaModel* in_models, int modelsCount, bool collectData);
	void freeModels(bool collectData = true);

	// Compute coords
	void cudaMVC(PRECISION3& point, 
				 PRECISION* weights,
				 int modelId,
				 int pointId,
				 bool collectData = true);

	double Precompute_Distances_CUDA(double* weights,
									 int indSize,
									 int* indirection, 
									 double threshold);

};

void mvcAllBindings(PRECISION3 point, PRECISION* weights, cudaModel* modelo);

int mainCuda();
*/
#endif
