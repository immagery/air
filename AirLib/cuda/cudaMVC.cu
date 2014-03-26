#include "cudaMVC.cuh"

#include <helper_cuda.h>
#include <texture_fetch_functions.h>

#define LITTLE_VALUE 0.0000001

PRECISION cudaSign(PRECISION v)
{
    if(v >= 0)
        return 1;
    else
        return -1;
}

__device__ PRECISION cSign(PRECISION v)
{
    if(v >= 0)
        return 1;
    else
        return -1;
}

__device__ PRECISION cDet(PRECISION3 u1, PRECISION3 u2, PRECISION3 u3)
{
    return u1.x*u2.y*u3.z + u2.x*u1.z*u3.y + u3.x*u1.y*u2.z
            - u1.z*u2.y*u3.x - u2.x*u1.y*u3.z - u3.y*u1.x*u2.z ;
}

__device__ double cDet(double3 u1, double3 u2, double3 u3)
{
    return u1.x*u2.y*u3.z + u2.x*u1.z*u3.y + u3.x*u1.y*u2.z
            - u1.z*u2.y*u3.x - u2.x*u1.y*u3.z - u3.y*u1.x*u2.z ;
}

__device__ float norm(float3& pt1)
{
	return sqrtf(pt1.x*pt1.x+pt1.y*pt1.y+pt1.z*pt1.z);
}

__device__ double norm(double3& pt1)
{
	return sqrt(pt1.x*pt1.x+pt1.y*pt1.y+pt1.z*pt1.z);
}

__device__ PRECISION3 divide_Pt_f(PRECISION3& pt1, PRECISION val)
{
	PRECISION3 ret = pt1;
	ret.x = ret.x /val;
	ret.y = ret.y /val;
	ret.z = ret.z /val; 

	return ret;
}

__device__ void divide_on_Pt_f(PRECISION3& pt1, PRECISION val)
{
	pt1.x = pt1.x /val;
	pt1.y = pt1.y /val;
	pt1.z = pt1.z /val;
}

__device__ PRECISION sq_norm(PRECISION3& pt1)
{
	return pt1.x*pt1.x+pt1.y*pt1.y+pt1.z*pt1.z;
}

// compute the unit vectors of each vertex to the point
__global__ void unitVectorsComputation( int nv,
										PRECISION3 point,
										cudaTextureObject_t nodePos, 	
										PRECISION3* unitVectors,  
										PRECISION* normas,
										PRECISION* weights,
										int* lockVertexes)
{
	int vertIt = threadIdx.x + blockIdx.x * blockDim.x;

	PRECISION3 tempUnit;
	while(vertIt < nv)
	{
		weights[vertIt] = 0.0;
		lockVertexes[vertIt] = 0;

		//unitVectors[vertIt].x = point.x - nodePos[vertIt].x;
		//unitVectors[vertIt].y = point.y - nodePos[vertIt].y;
		//unitVectors[vertIt].z = point.z - nodePos[vertIt].z;

		tempUnit.x = tex1Dfetch<PRECISION>(nodePos, vertIt*3) - point.x;
		tempUnit.y = tex1Dfetch<PRECISION>(nodePos, vertIt*3+1) - point.y;
		tempUnit.z = tex1Dfetch<PRECISION>(nodePos, vertIt*3+2) - point.z;

		PRECISION n = norm(tempUnit);

		if(n > LITTLE_VALUE)
		{
			tempUnit.x /= n;
			tempUnit.y /= n;
			tempUnit.z /= n;
		}
		else
		{
			weights[vertIt] = 1.0;
			return;
		}

		unitVectors[vertIt].x = tempUnit.x;
		unitVectors[vertIt].y = tempUnit.y;
		unitVectors[vertIt].z = tempUnit.z;
		normas[vertIt] = n/1000.0;

		vertIt += blockDim.x*gridDim.x;
	}
}

__global__ void coordsComputation( int nt, 
								   PRECISION3 point,
								   cudaTextureObject_t triangles,  	
								   cudaTextureObject_t unitVectors,
								   cudaTextureObject_t normas,
								   int* lockVertexes,
								   PRECISION* weights)
{

	int fj = threadIdx.x + blockIdx.x * blockDim.x;

	double O[3], c[3], s[3], l[3];
	unsigned int idVerts[3];

	while(fj < nt)
	{
		idVerts[0] = tex1Dfetch<int>(triangles, fj*3+0);
		idVerts[1] = tex1Dfetch<int>(triangles, fj*3+1);
		idVerts[2] = tex1Dfetch<int>(triangles, fj*3+2);

		for(int i = 0; i<3; i++)
		{
			int id1 = (i+1)%3;
			int id2 = (i+2)%3;

			double3 rest;
			rest.x = tex1Dfetch<PRECISION>(unitVectors, idVerts[id1]*3); 
			rest.y = tex1Dfetch<PRECISION>(unitVectors, idVerts[id1]*3+1); 
			rest.z = tex1Dfetch<PRECISION>(unitVectors, idVerts[id1]*3+2); 

			rest.x -= tex1Dfetch<PRECISION>(unitVectors, idVerts[id2]*3);
			rest.y -= tex1Dfetch<PRECISION>(unitVectors, idVerts[id2]*3+1);
			rest.z -= tex1Dfetch<PRECISION>(unitVectors, idVerts[id2]*3+2);

			l[i] = norm(rest);
			O[i] = 2.0*asin(l[i]/2.0);
		}

		double h = (O[0]+O[1]+O[2])/2;

		if(CUDART_PI - h < tresh2) // x esta sobre el triangulo t, usar coords bar 2D.
		{
			for(int i = 0; i<3; i++)
			{
				bool alocated = false;
				while(!alocated)
				{
					if( __iAtomicCAS(&lockVertexes[idVerts[i]], 0, 1) == 0 )
					{
						weights[idVerts[i]] += (float)(sin(O[i])* l[(i+2)%3] * l[(i+1)%3]);

						atomicExch(&lockVertexes[idVerts[i]],0);
						alocated = true;
					}
				}
			}

			return;
		}
		
		double3 v[3];
		for(int i = 0; i< 3; i++)
		{
			v[i].x = tex1Dfetch<PRECISION>(unitVectors, idVerts[i]*3);
			v[i].y = tex1Dfetch<PRECISION>(unitVectors, idVerts[i]*3+1);
			v[i].z = tex1Dfetch<PRECISION>(unitVectors, idVerts[i]*3+2);
		}

		//PRECISION determ = cDet(unitVectors[idVerts[0]], unitVectors[idVerts[1]], unitVectors[idVerts[2]]);
		double determ = cDet(v[0], v[1], v[2]);
		
		bool okValues = true;
		for(int i = 0; i<3; i++)
		{
			double num = 2*sin(h)*sin(h-O[i]);
			double denom = sin(O[(i+1)%3])*sin(O[(i+2)%3]);
			c[i] = num/denom-1;

			s[i] = sqrt(1-c[i]*c[i]);
			if(determ < 0) // Cambiamos el signo
				s[i] = -s[i];
			
			okValues &= (fabs(s[i]) > tresh3);
		}

		//PRECISION w[3];
		if(okValues)
		{
			//weights[fj*3]   = (O[0]- c[1]*O[2] - c[2]*O[1])/(normas[idVerts[0]]*sin(O[1])*s[2]);
			//weights[fj*3+1] = (O[1]- c[0]*O[2] - c[2]*O[0])/(normas[idVerts[1]]*sin(O[2])*s[0]);
			//weights[fj*3+2] = (O[2]- c[1]*O[0] - c[0]*O[1])/(normas[idVerts[2]]*sin(O[0])*s[1]);

			for(int i = 0; i< 3; i++)
			{
				int id1 = (i+1)%3;
				int id2 = (i+2)%3;

				bool alocated = false;
				double norm = (double)tex1Dfetch<PRECISION>(normas, idVerts[i]);

				while(!alocated)
				{
					if( __iAtomicCAS(&lockVertexes[idVerts[i]], 0, 1) == 0 )
					{
						weights[idVerts[i]] += (O[i]- c[id1]*O[id2] - c[id2]*O[id1])/(norm*sin(O[id1])*s[id2]);
						atomicExch(&lockVertexes[idVerts[i]],0);
						alocated = true;
					}
				}
			}

			//weights[pos[0]] = (O[0]- c[1]*O[2] - c[2]*O[1])/(tex1Dfetch<PRECISION>(normas, idVerts[0])*sin(O[1])*s[2]);
			//weights[pos[1]] = (O[1]- c[0]*O[2] - c[2]*O[0])/(tex1Dfetch<PRECISION>(normas, idVerts[1])*sin(O[2])*s[0]);
			//weights[pos[2]] = (O[2]- c[1]*O[0] - c[0]*O[1])/(tex1Dfetch<PRECISION>(normas, idVerts[2])*sin(O[0])*s[1]);
		}

		fj += blockDim.x*gridDim.x;
	}
	
	return;
}

__global__ void sumWeightsParcial(int* lockVertexes,
								  PRECISION* tempWeights, 
								  int ntri, 
								  int npts, 
								  cudaTextureObject_t triangleIdx, 
								  PRECISION* weights)
{
	int tid = threadIdx.x + blockIdx.x * blockDim.x;

	PRECISION temp = 0;
	if(tid < ntri)
	{
		temp = 0;
		int k = 0;
		for( k = 0; k< 3; k++)
		{
			int idx = tex1Dfetch<int>(triangleIdx, tid*3+k);
			temp += tempWeights[tid*3+k];
			bool alocated = false;

			while(!alocated)
			{
				if( __iAtomicCAS(&lockVertexes[idx], 0, 1) == 0 )
				{
					weights[idx] += tempWeights[tid*3+k];
					atomicExch(&lockVertexes[idx],0);
					alocated = true;
				}
			}
		}

		//tempWeights[tid*3] = temp;
		tid += blockDim.x*gridDim.x;
	}
}

__global__ void sumWeightsRedux(int* lock,
								PRECISION* tempWeights, 
								int npts, 
								PRECISION* cudaSum)
{
	__shared__ double cache[threadsPerBlock];

	int tid = threadIdx.x + blockIdx.x * blockDim.x;
	int cacheIndex = threadIdx.x;
	
	double temp = 0.0;
	while(tid < npts)
	{
		temp += tempWeights[tid];
		tid += blockDim.x*gridDim.x;
	}

	cache[cacheIndex] = temp;

	__syncthreads();

	int i = blockDim.x/2;
	while(i!=0)
	{
		if(cacheIndex < i)
			cache[cacheIndex] += cache[cacheIndex+i];

		__syncthreads();
		i/=2;
	}

	if(cacheIndex == 0)
	{
		//Bloquear la escritura si hay conflicto
		int ret = __iAtomicCAS(lock, 0, 1);
		while( ret > 0 )
		{
			ret = __iAtomicCAS(lock, 0, 1);
		}
		
		*cudaSum += (float)cache[0];
		
		atomicExch(lock,0);
		
	}

	/*
	__syncthreads();

	// Realizamos la normalizacion
	float sum = *cudaSum;
	int tid2 = threadIdx.x + blockIdx.x * blockDim.x;
	while(tid2 < npts)
	{
		tempWeights[tid2] = tempWeights[tid2]/sum;
		tid2 += blockDim.x*gridDim.x;
	}
	*/
}

__global__ void normalizeWeights(PRECISION* weights, PRECISION* sum, int nv)
{
	int tid = threadIdx.x + blockIdx.x*blockDim.x;
	
	float fsum = (*sum);
	while(tid < nv)
	{
		weights[tid] = weights[tid]/ fsum;
		tid += blockDim.x * gridDim.x;
	}
}


__global__ void sortWeights(PRECISION* weights, PRECISION* tw01, PRECISION* tw02, int* ind, int npts, int* cuttingPt)
{

	/*
	int tid = threadIdx.x + blockIdx.x*blockDim.x;
	
	float fsum = (*sum);
	while(tid < nv)
	{
		weights[tid] = weights[tid]/ fsum;
		tid += blockDim.x * gridDim.x;
	}
	*/
}
