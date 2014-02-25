#ifndef CUDA_MVC_H
#define CUDA_MVC_H

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

class cudaModel
{
public:
	double3* nodes;
	int3* triangles;
	float** coords;

	unsigned int nv;
	unsigned int nt;

	unsigned int getMemorySize()
	{
		unsigned int sum = nv*nv/2;
		
		return nv*sizeof(double3) + 
			   nt*sizeof(int3) + 
			   sum*sizeof(float) + 
			   sizeof(unsigned int)*2;
	}
};

class cudaManager
{
public:

	cudaManager()
	{
		nodePositions = NULL;
		cudaTriangles = NULL;

		cudaNormas = NULL;
		cudaUnitVectors = NULL;
		cudaCoords = NULL;
		tempResults = NULL;

		cudaWeights = NULL;


		ntri = npts = 0; 

		loaded = false;
	}

	// Model
	double3* nodePositions;
	int3* cudaTriangles;

	int npts;
	int ntri;

	// Temporal data
	double* cudaNormas;
	double3* cudaUnitVectors;

	double* resWeights;
	double* cudaWeights;

	double* cudaCoords;
	double* tempResults;

	bool loaded;
	int blocksPerGrid;

	// Allocate and load data
	void loadModel(cudaModel& modelo);

	void freeModel();

	// Compute coords
	void cudaMVC(double3& point, 
			 double* weights, 
			 cudaModel& modelo);
};

void mvcAllBindings(double3 point, double* weights, cudaModel* modelo);

int mainCuda();

#endif