#include "cudaMVC.cuh"

#include <assert.h>
#include <helper_cuda.h>


#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <device_functions.h>
#include <math_constants.h>

#include <time.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define M_PI 3.14159265
#define tresh1 0.0001
#define tresh2 0.001
#define tresh3 0.0001

const int threadsPerBlock = 256;

cudaError_t addWithCuda(int *c, const int *a, const int *b, unsigned int size);

__global__ void addKernel(int *c, const int *a, const int *b)
{
    int i = threadIdx.x;
    c[i] = a[i] + b[i];
}

double cudaSign(double v)
{
    if(v >= 0)
        return 1;
    else
        return -1;
}

double det(double3 u1, double3 u2, double3 u3)
{
    return u1.x*u2.y*u3.z + u2.x*u1.z*u3.y + u3.x*u1.y*u2.z
            - u1.z*u2.y*u3.x - u2.x*u1.y*u3.z - u3.y*u1.x*u2.z ;
}


__device__ double cSign(double v)
{
    if(v >= 0)
        return 1;
    else
        return -1;
}

__device__ double cDet(double3 u1, double3 u2, double3 u3)
{
    return u1.x*u2.y*u3.z + u2.x*u1.z*u3.y + u3.x*u1.y*u2.z
            - u1.z*u2.y*u3.x - u2.x*u1.y*u3.z - u3.y*u1.x*u2.z ;
}

/*
__device__ double3& rest(double3& pt1, double3& pt2)
{
	double3 ret;
	ret.x = pt1.x - pt2.x;
	ret.y = pt1.y - pt2.y;
	ret.z = pt1.z - pt2.z;

	return ret;
}
*/

__device__ double norm(double3& pt1)
{
	return sqrtf(pt1.x*pt1.x+pt1.y*pt1.y+pt1.z*pt1.z);
}

__device__ double3 divide_Pt_f(double3& pt1, float val)
{
	double3 ret = pt1;
	ret.x = ret.x /val;
	ret.y = ret.y /val;
	ret.z = ret.z /val;

	return ret;
}

__device__ void divide_on_Pt_f(double3& pt1, float val)
{
	pt1.x = pt1.x /val;
	pt1.y = pt1.y /val;
	pt1.z = pt1.z /val;
}

__device__ float sq_norm(double3& pt1)
{
	return pt1.x*pt1.x+pt1.y*pt1.y+pt1.z*pt1.z;
}

// compute the unit vectors of each vertex to the point
__global__ void unitVectorsComputation( int nv,
										double3 point,
										double3* nodePos,   	
										double3* unitVectors,  
										double* normas)
{
	int vertIt = threadIdx.x + blockIdx.x * blockDim.x;

	while(vertIt < nv)
	{
		unitVectors[vertIt].x = point.x - nodePos[vertIt].x;
		unitVectors[vertIt].y = point.y - nodePos[vertIt].y;
		unitVectors[vertIt].z = point.z - nodePos[vertIt].z;

		double n = norm(unitVectors[vertIt]);

		unitVectors[vertIt].x /= n;
		unitVectors[vertIt].y /= n;
		unitVectors[vertIt].z /= n;

		normas[vertIt] = n;

		vertIt += blockDim.x*gridDim.x;
	}
}

__global__ void coordsComputation( int nt, 
								   double3 point,
								   double3* nodePos,   	
								   int3* triangles,
								   double3* unitVectors,  
								   double* normas,
								   double* weights)
{

	int fj = threadIdx.x + blockIdx.x * blockDim.x;

	double totalW;
	double O[3], c[3], s[3];
	int idVerts[3];

	while(fj < nt)
	{
		totalW = 0;
		idVerts[0] = triangles[fj].x;
		idVerts[1] = triangles[fj].z;
		idVerts[2] = triangles[fj].y;

		for(int i = 0; i<3; i++)
		{
			int id1 = (i+1)%3;
			int id2 = (i+2)%3;

			double3 rest = unitVectors[idVerts[id1]];
			rest.x -= unitVectors[idVerts[id2]].x;
			rest.y -= unitVectors[idVerts[id2]].y;
			rest.z -= unitVectors[idVerts[id2]].z;

			double l = norm(rest);
			O[i] = 2*asin(l/2);
		}

		double h = (O[0]+O[1]+O[2])/2;

		if(CUDART_PI - h < tresh2) // x esta sobre el triangulo t, usar coords bar 2D.
		{
			for(int i = 0; i<3; i++)
			{
				weights[fj*3+i] = sin(O[i])*normas[idVerts[(i+1)%3]]*normas[idVerts[(i+2)%3]];
				totalW += weights[fj*3+i];
			}
		
			for(int i = 0; i<3 && totalW > 0; i++)
			{
				weights[fj*3+i] /= totalW;
			}

			return;
		}

		double determ = cDet(unitVectors[idVerts[0]], unitVectors[idVerts[1]], unitVectors[idVerts[2]]);

		bool okValues = true;
		for(int i = 0; i<3; i++)
		{

			int idx1 = (i+1)%3;
			int idx2 = (i+2)%3;
			double num = 2*sin(h)*sin(h-O[i]);
			double denom = sin(O[idx1])*sin(O[idx2]);
			c[i] = num/denom-1;

			s[i] = sqrt(1-c[i]*c[i]);
			if(determ < 0)
				s[i] = -sqrt(1-c[i]*c[i]);
			
			okValues &= (fabs(s[i]) > tresh3);
		}

		double w[3];
		if(okValues)
		{
			weights[fj*3]   = (O[0]- c[1]*O[2] - c[2]*O[1])/(normas[idVerts[0]]*sin(O[1])*s[2]);
			weights[fj*3+1] = (O[1]- c[0]*O[2] - c[2]*O[0])/(normas[idVerts[1]]*sin(O[2])*s[0]);
			weights[fj*3+2] = (O[2]- c[1]*O[0] - c[0]*O[1])/(normas[idVerts[2]]*sin(O[0])*s[1]);
		}

		fj += blockDim.x*gridDim.x;
	}
	
	return;
}

void cudaManager::loadModel(cudaModel& modelo)
{
	cudaDeviceProp prop;
	int whichDevice;

	checkCudaErrors( cudaGetDevice( &whichDevice ) );
	checkCudaErrors( cudaGetDeviceProperties( &prop, whichDevice) );

	if(prop.canMapHostMemory != 1)
	{
		printf("Device cannot map memory.\n");
		return;
	}

	checkCudaErrors( cudaSetDeviceFlags( cudaDeviceMapHost ) );

	npts = modelo.nv;
	ntri = modelo.nt;

	blocksPerGrid = (npts+threadsPerBlock-1)/threadsPerBlock;
	if(blocksPerGrid > 32) blocksPerGrid = 32;

	// allocate the memory on the GPU
	checkCudaErrors( cudaMalloc( (void**)&cudaUnitVectors, npts * sizeof(double3) ) );
	checkCudaErrors( cudaMalloc( (void**)&cudaNormas, npts * sizeof(double) ) );
	checkCudaErrors( cudaMalloc( (void**)&nodePositions, npts * sizeof(double3) ) );
	checkCudaErrors( cudaMalloc( (void**)&cudaTriangles, ntri * sizeof(double3) ) );

	checkCudaErrors( cudaMalloc( (void**)&cudaWeights, ntri * sizeof(double3) ) );
	//checkCudaErrors( cudaMalloc( (void**)&cudaCoords, npts * sizeof(double) ) );

	//tempResults = (double*) malloc(blocksPerGrid*sizeof(double));

	//checkCudaErrors( cudaHostAlloc( (void**)&resWeights, 
	//								ntri * sizeof(double) *3, 
	//								cudaHostAllocMapped) );
	
	//for(int i = 0; i< ntri*3; i++)
	//	resWeights[i] = 0.0;
								
	checkCudaErrors( cudaHostAlloc( (void**)&tempResults, 
									npts * sizeof(double), 
									cudaHostAllocMapped) );

	for(int i = 0; i< npts; i++)
		tempResults[i] = 0.0;

	//checkCudaErrors(cudaHostGetDevicePointer(&cudaWeights, resWeights, 0));
	checkCudaErrors(cudaHostGetDevicePointer(&cudaCoords, tempResults, 0));


	// Copy data to GPU
	checkCudaErrors( cudaMemcpy(nodePositions, modelo.nodes, npts * sizeof(double3), cudaMemcpyHostToDevice) );
	checkCudaErrors( cudaMemcpy(cudaTriangles, modelo.triangles, ntri * sizeof(int3), cudaMemcpyHostToDevice) );

	cudaError_t cudaStatus = cudaGetLastError();
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "memory allocation for mvc: %s\n", cudaGetErrorString(cudaStatus));
        fflush(0);
		return;
    }

	loaded = true;

}

void cudaManager::freeModel()
{	
	// device Memory
	cudaFree(cudaTriangles);
	cudaFree(cudaUnitVectors);
	cudaFree(cudaNormas);
	cudaFree(nodePositions);

	// Pined-mapped memory
	cudaFreeHost(tempResults);
	//cudaFreeHost(resWeights);

	cudaFree(cudaWeights);
	//cudaFree(cudaCoords);

	loaded = false;
}

__global__ void sumWeightsRedux(double* weights, double* tempSum, int nv)
{
	__shared__ double cache[threadsPerBlock];

	int tid = threadIdx.x + blockIdx.x * blockDim.x;
	int cacheIndex = threadIdx.x;

	double temp = 0;
	while(tid < nv)
	{
		temp += weights[tid*3] + weights[tid*3+1] + weights[tid*3+2];
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
		tempSum[blockIdx.x] = cache[0];
}

__global__ void normalizeWeights(double* weights, int nv, double sum)
{
	int tid = threadIdx.x + blockIdx.x*blockDim.x;

	while(tid < nv)
	{
		weights[tid*3] = weights[tid*3]/sum;
		weights[tid*3+1] = weights[tid*3+1]/sum;
		weights[tid*3+2] = weights[tid*3+2]/sum;

		tid += blockDim.x * gridDim.x;
	}
}

double timeLapseCuda(clock_t clock1,clock_t clock2)
{
	double diffticks=clock2-clock1;
    double diffms=diffticks/CLOCKS_PER_SEC;
    return diffms;
}

// PREMISAS -> Hay que asegurar todo esto desde fuera
void cudaManager::cudaMVC(double3& point,  double* weights,  cudaModel& modelo)
{
	
	//clock_t iniTotal = clock();

	//clock_t ini = clock();
	unitVectorsComputation<<< blocksPerGrid, threadsPerBlock >>>(npts, point, nodePositions, cudaUnitVectors, cudaNormas);
	checkCudaErrors( cudaThreadSynchronize() );


	//clock_t fin = clock();
	//printf("TIEMPOS----\n unitVectorsComputation: %f, total:%f\n", timeLapseCuda(ini,fin)*1000, timeLapseCuda(iniTotal,fin)*1000);
	/*
	cudaError_t cudaStatus = cudaGetLastError();
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "unitVectorsComputation fallo: %s\n", cudaGetErrorString(cudaStatus));
        fflush(0);
		return;
    }
	*/

	// FOR TEST NORMALS -> test float to double
	/*double* hostNorms = (double*)malloc(npts*sizeof(double));
	checkCudaErrors( cudaMemcpy(hostNorms, cudaNormas, npts * sizeof(double), cudaMemcpyDeviceToHost) );

	double3* hostVects = (double3*)malloc(npts*sizeof(double3));
	checkCudaErrors( cudaMemcpy(hostVects, cudaUnitVectors, npts * sizeof(double3), cudaMemcpyDeviceToHost) );

	printf("___unitVectors___\n");
	for(int i = 0; i< npts; i++)
		printf(" %f", hostNorms[i]);
	printf("\n");

	free(hostNorms);
	

	for(int i = 0; i < 10; i++) 
		printf(" CUDA - v.x: %f, v.y: %f, v.z: %f, n:%f\n", hostVects[i].x,hostVects[i].y, hostVects[i].z, hostNorms[i]);
	free(hostNorms);
	free(hostVects);
	*/

	//ini = clock();
	coordsComputation<<< blocksPerGrid, threadsPerBlock >>>(ntri, point, nodePositions, cudaTriangles, 
															 cudaUnitVectors, cudaNormas, cudaWeights);
	checkCudaErrors( cudaThreadSynchronize() );
	//fin= clock();
	//printf("coordsComputation: %f, total:%f\n", timeLapseCuda(ini,fin)*1000, timeLapseCuda(iniTotal,fin)*1000);
	/*
	cudaStatus = cudaGetLastError();
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "coordsComputation failed: %s\n", cudaGetErrorString(cudaStatus));
        fflush(0);
		return;
    }
	*/

	//for(int i = 0; i < 30; i++) 
	//	printf(" CUDA - w[0]: %f, w[1]: %f, w[2]: %f\n", resWeights[i*3], resWeights[i*3+1], resWeights[i*3+2]);
	
	//double* hostWeights = (double*)malloc(ntri*sizeof(double)*3);
	//checkCudaErrors( cudaMemcpy(hostWeights, cudaWeights, ntri * sizeof(double)*3, cudaMemcpyDeviceToHost) );

	//free(hostWeights);

	//printf("BlocksPerGrid: %f, ThreadsPerBlock: %d\n", blocksPerGrid, threadsPerBlock);

	//ini = clock();
	//float* cudaWeightsTempSum;
	//checkCudaErrors( cudaMalloc( (void**)&cudaWeightsTempSum, blocksPerGrid * sizeof(float) ) );
	//fin= clock();
	//printf("memory alloc: %f\n", timeLapseCuda(ini,fin));

	//ini = clock();
	sumWeightsRedux<<< blocksPerGrid, threadsPerBlock >>>(cudaWeights, cudaCoords, ntri);
	
	checkCudaErrors( cudaThreadSynchronize() );

	//fin= clock();
	//printf("sumWeightsRedux: %f, total:%f\n", timeLapseCuda(ini,fin)*1000, timeLapseCuda(iniTotal,fin)*1000);

	//ini = clock();
	//checkCudaErrors( cudaMemcpy(tempResults, cudaCoords, blocksPerGrid * sizeof(double), cudaMemcpyDeviceToHost));
	// Sum just the bloccks... I hope that this could be also impoved.
	//fin = clock();
	//printf("copia_memoria_en_local: %f, %d elementos\n", timeLapseCuda(ini,fin)*1000,blocksPerGrid);
	
	//ini = clock();
	double sum = 0;
	for(int i = 0; i< blocksPerGrid; i++)
		sum+=tempResults[i]; // sumando cudaCoords

	/*
	printf("___weights_without_normalize___\n");
	for(int i = 0; i< blocksPerGrid; i++)
		printf(" %f ", tempResults[i]);
	printf("\n");
	printf(" sum: %f\n", sum);
	*/
	//cudaFree(cudaCoords);
	//fin= clock();
	//printf("suma_intermedia: %f, total:%f\n", timeLapseCuda(ini,fin)*1000, timeLapseCuda(iniTotal,fin)*1000);

	//ini = clock();
	normalizeWeights<<< blocksPerGrid, threadsPerBlock >>>(cudaWeights, ntri, sum);
	checkCudaErrors( cudaThreadSynchronize() );
	//fin= clock();
	//printf("normalizeWeights: %f, total:%f\n", timeLapseCuda(ini,fin)*1000, timeLapseCuda(iniTotal,fin)*1000);
	//ini = clock();

	/*
	for(int i = 0; i< npts; i++)
		weights[i] = 0.0;


	for(int i = 0; i< ntri; i++)
	{
		int idVert1 = modelo.triangles[i].x;
		int idVert2 = modelo.triangles[i].y;
		int idVert3 = modelo.triangles[i].z;

		weights[idVert1] += resWeights[i*3];
		weights[idVert2] += resWeights[i*3+1];
		weights[idVert3] += resWeights[i*3+2];
	}
	*/

	//fin= clock();
	//printf("sincronize thread: %f, total:%f\n", timeLapseCuda(ini,fin)*1000, timeLapseCuda(iniTotal,fin)*1000);

	//ini = clock();
	//checkCudaErrors( cudaMemcpy(resWeights, cudaWeights, npts * sizeof(float), cudaMemcpyDeviceToHost) );
	//fin= clock();
	//printf("recogerDatos: %f\n", timeLapseCuda(ini,fin)*1000);

	//clock_t finTotal = clock();
	//printf("\n\nTOTAL: %f\n", timeLapseCuda(iniTotal,finTotal)*1000);

}

void mvcAllBindings(double3& point, double* weights, cudaModel* modelo)
{
	/*
    //const double tresh1 = 0.0001;
    //const double tresh2 = 0.001;
    //const double tresh3 = 0.0001;

	// We use all the model
	int nopts = modelo->nv;
    if(nopts == 0)
    {
        printf("ERROR!: El modelo no está bien inicializado!\n");
        return;
    }   

	weights.clear();
    weights.resize(nopts,0.0); // incializamos el resultado

    vector< cudaPoint3d> unitVectors;
    vector<double> normas;
    unitVectors.resize(nopts);
    normas.resize(nopts);

    //MyMesh::VertexIterator vertIt;
    //for(vertIt = modelo.vert.begin(); vertIt!=modelo.vert.end(); ++vertIt )
	for(int vertIt = 0; vertIt < modelo->nv; vertIt++ )
	{
		//Vector3d dirVec = vertIt->P() - point;
		cudaPoint3d dirVec = point - cudaPoint3d(modelo->nodes[vertIt]);
        double norm = dirVec.norm();
		int idVert = vertIt;//modelo->nodes[vertIt].Id;

        //assert(idVert >= 0 && idVert < nopts); // Comprobaci—n de los valors de IMark
		
        if(norm < tresh1)
        {
            // El punto est‡ muy cerca de este vŽrtice, devolvemos los valores en V tal cual.
			weights[idVert] = 1.0;
            return; // Ya no tenemos que calcular nada m‡s para este punto.
        }
            
        unitVectors[idVert] = dirVec/norm;
        normas[idVert] = norm;
    }

    double totalW = 0;

    //MyMesh::FaceIterator fj;
    //for(fj = modelo.face.begin(); fj!=modelo.face.end(); ++fj )
	for(unsigned int fj = 0; fj < modelo->nv; fj++ )
	{
         cudaPoint3d O, c, s;
         cudaPoint3i idVerts;
        //for(int i = 0; i<3; i++) // Obtenemos los indices de los vertices de t
			//idVerts[i] = fj->V(i)->IMark();
			//idVerts[2-i] = fj->V(i)->IMark();

		idVerts.x = modelo->triangles[fj].x;
		idVerts.y = modelo->triangles[fj].y;
		idVerts.z = modelo->triangles[fj].z;

        for(int i = 0; i<3; i++)
        {
            double l = (unitVectors[idVerts[(i+1)%3]]- unitVectors[idVerts[(i+2)%3]]).norm();
            O[i] = 2*asin(l/2);
        }

        double h = (O.x+O.y+O.z)/2;

		cudaPoint3d w; double w_sum = 0; // espacio para coordenadas y la suma

        if(M_PI - h < tresh2) // x esta sobre el triangulo t, usar coords bar 2D.
        {
            for(int i = 0; i<3; i++){  // Calculamos los valores de las coordenadas y la suma
                w[i] = sin(O[i])*normas[idVerts[(i+1)%3]]*normas[idVerts[(i+2)%3]];
                w_sum += w[i];
            }

			for(int i = 0; i<3; i++)
			{  // Guardamos la coordenada ponderada por la suma: hay que preservar la particion de unidad.
				weights[idVerts[i]] = w[i]/w_sum;
			}

            return; // Acabamos
        }

		double determ = det(unitVectors[idVerts[0]], unitVectors[idVerts[1]], unitVectors[idVerts[2]]);

        bool okValues = true;
        for(int i = 0; i<3 && okValues; i++)
        {
            c[i] = 2*sin(h)*sin(h-O[i])/(sin(O[(i+1)%3])*sin(O[(i+2)%3]))-1;
            s[i] = cudaSign(determ)*sqrt(1-c[i]*c[i]);

            okValues &= (fabs(s[i]) > tresh3);
        }

        if(!okValues)
            continue;

        w[0] = (O[0]- c[1]*O[2] - c[2]*O[1])/(normas[idVerts[0]]*sin(O[1])*s[2]);
        w[1] = (O[1]- c[0]*O[2] - c[2]*O[0])/(normas[idVerts[1]]*sin(O[2])*s[0]);
        w[2] = (O[2]- c[1]*O[0] - c[0]*O[1])/(normas[idVerts[2]]*sin(O[0])*s[1]);

		for(int i = 0; i<3; i++)
		{  
			// Guardamos la coordenada ponderada por la suma: hay que preservar la particion de unidad.
			weights[idVerts[i]] += w[i];		
			totalW +=w[i];
		}
    }

	double sum2 = 0;
	for(int i = 0; i< weights.size(); i++)
    {
		sum2 += weights[i];
	}

	for(int i = 0; i< weights.size(); i++)
	{
		double auxWeight = weights[i];
		weights[i] = weights[i]/sum2;
	}
	*/
}


int mainCuda()
{
    const int arraySize = 5;
    const int a[arraySize] = { 1, 2, 3, 4, 5 };
    const int b[arraySize] = { 10, 20, 30, 40, 50 };
    int c[arraySize] = { 0 };

    // Add vectors in parallel.
    cudaError_t cudaStatus = addWithCuda(c, a, b, arraySize);
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "addWithCuda failed!");
        return 1;
    }

    printf("{1,2,3,4,5} + {10,20,30,40,50} = {%d,%d,%d,%d,%d}\n",
        c[0], c[1], c[2], c[3], c[4]);

    // cudaDeviceReset must be called before exiting in order for profiling and
    // tracing tools such as Nsight and Visual Profiler to show complete traces.
    cudaStatus = cudaDeviceReset();
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaDeviceReset failed!");
        return 1;
    }

    return 0;
}

// Helper function for using CUDA to add vectors in parallel.
cudaError_t addWithCuda(int *c, const int *a, const int *b, unsigned int size)
{
    int *dev_a = 0;
    int *dev_b = 0;
    int *dev_c = 0;
    cudaError_t cudaStatus;

    // Choose which GPU to run on, change this on a multi-GPU system.
    cudaStatus = cudaSetDevice(0);
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaSetDevice failed!  Do you have a CUDA-capable GPU installed?");
        goto Error;
    }

    // Allocate GPU buffers for three vectors (two input, one output)    .
    cudaStatus = cudaMalloc((void**)&dev_c, size * sizeof(int));
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMalloc failed!");
        goto Error;
    }

    cudaStatus = cudaMalloc((void**)&dev_a, size * sizeof(int));
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMalloc failed!");
        goto Error;
    }

    cudaStatus = cudaMalloc((void**)&dev_b, size * sizeof(int));
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMalloc failed!");
        goto Error;
    }

    // Copy input vectors from host memory to GPU buffers.
    cudaStatus = cudaMemcpy(dev_a, a, size * sizeof(int), cudaMemcpyHostToDevice);
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMemcpy failed!");
        goto Error;
    }

    cudaStatus = cudaMemcpy(dev_b, b, size * sizeof(int), cudaMemcpyHostToDevice);
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMemcpy failed!");
        goto Error;
    }

    // Launch a kernel on the GPU with one thread for each element.
    addKernel<<<1, size>>>(dev_c, dev_a, dev_b);

    // Check for any errors launching the kernel
    cudaStatus = cudaGetLastError();
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "addKernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
        goto Error;
    }
    
    // cudaDeviceSynchronize waits for the kernel to finish, and returns
    // any errors encountered during the launch.
    cudaStatus = cudaDeviceSynchronize();
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching addKernel!\n", cudaStatus);
        goto Error;
    }

    // Copy output vector from GPU buffer to host memory.
    cudaStatus = cudaMemcpy(c, dev_c, size * sizeof(int), cudaMemcpyDeviceToHost);
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMemcpy failed!");
        goto Error;
    }

Error:
    cudaFree(dev_c);
    cudaFree(dev_a);
    cudaFree(dev_b);
    
    return cudaStatus;
}
