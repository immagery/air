#include "cudaBubbleSort.cuh"
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#include <ctime>
#include <stdio.h>
#include <vector>
#include <limits>
#include <algorithm>

#include <Windows.h>

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>

#include <iostream>

__host__ int64 GetTimeMs64()
{
	 /* Windows */
	 FILETIME ft;
	 LARGE_INTEGER li;

	 /* Get the amount of 100 nano seconds intervals elapsed 
	  * since January 1, 1601 (UTC) and copy it
	  * to a LARGE_INTEGER structure. */
	 GetSystemTimeAsFileTime(&ft);
	 li.LowPart = ft.dwLowDateTime;
	 li.HighPart = ft.dwHighDateTime;

	 uint64 ret = li.QuadPart;
	 ret -= 116444736000000000LL; 
	 /* Convert from file time to UNIX epoch time. */
	 ret /= 10000; /* From 100 nano seconds (10^-7) 
				   to 1 millisecond (10^-3) intervals */

	 return ret;
}

/*
__global__ void sumAndGetThreshold(float *a, float *aCopy, 
									int* ind, float* negSum, 
									int* lock, int size, 
									int* cuttingIdx, float *posSum)
{
	int elem = threadIdx.x + blockDim.x * blockIdx.x;
	while(elem < size)
	{
		aCopy[elem] = a[elem];
		ind[elem] = elem;

		elem = elem + blockDim.x * gridDim.x;
	}

	__syncthreads();

	float cacheFirst = 0;
	float cacheSecond = 0;
	float cacheThird = 0;

	int cacheFirstInd;
	int cacheSecondInd;
	int cacheThirdInd ;

	for (int j = 0; j < size/2+1; j++) 
	{
		int i = (threadIdx.x + blockDim.x*blockIdx.x)*2;
		
		while(i+1 < size)
		{
			cacheFirst = aCopy[i];
			cacheSecond = aCopy[i+1];

			cacheFirstInd = ind[i];
			cacheSecondInd = ind[i+1];

			if(cacheFirst < cacheSecond) 
			{
				// Weight
				float temp = cacheFirst;
				aCopy[i] = cacheSecond;
				aCopy[i+1] = temp;

				// Indirection
				int tempInd = cacheFirstInd;
				ind[i] = cacheSecondInd;
				ind[i+1] = tempInd;
			}

			i += (blockDim.x*gridDim.x)*2;
		}

		__syncthreads();

		i = (threadIdx.x + blockDim.x*blockIdx.x)*2 +1;
	
		while(i+1 < size)
		{
			cacheFirst = aCopy[i];
			cacheSecond = aCopy[i+1];

			cacheFirstInd = ind[i];
			cacheSecondInd = ind[i+1];

			if(cacheFirst < cacheSecond) 
			{
				// Weight
				float temp = cacheFirst;
				aCopy[i] = cacheSecond;
				aCopy[i+1] = temp;

				// Indirection
				int tempInd = cacheFirstInd;
				ind[i] = cacheSecondInd;
				ind[i+1] = tempInd;
			}
			i += (blockDim.x*gridDim.x)*2;
		}
	}

	__syncthreads();

	__shared__ int thatPoint;
	__shared__ float cachePos[1024];
	__shared__ float cacheNeg[1024];
	__shared__ float sumNegShared;
    
	sumNegShared = 0.0;
	
	int i = blockDim.x * blockIdx.x + threadIdx.x;
	int tid = blockDim.x * blockIdx.x + threadIdx.x;
	int cacheIndex = threadIdx.x;
	
	float tempNeg = 0.0;
	float tempPos = 0.0;
	while(tid < size)
	{
		float aCopyTemp = aCopy[tid];
		if(aCopyTemp < 0.0)
			tempNeg += aCopyTemp;
		//else
		//	tempPos += aCopyTemp;

		tid += blockDim.x*gridDim.x;
	}

	//cachePos[cacheIndex] = tempPos;
    cacheNeg[cacheIndex] = tempNeg;

	__syncthreads();

	int NewI = blockDim.x/2;
	while(NewI!=0)
	{
		if(cacheIndex < NewI)
		{
			//cachePos[cacheIndex] += cachePos[cacheIndex+NewI];
			cacheNeg[cacheIndex] += cacheNeg[cacheIndex+NewI];
		}

		__syncthreads();
		NewI/=2;
	}

	if(cacheIndex == 0)
	{
		//*posSum = (float)cachePos[0];
		*negSum = cacheNeg[0];
		sumNegShared = cacheNeg[0];
	}
	
	__syncthreads();

	
	// Buscar el primer indice donde queda anulada la suma negativa.
	tid = blockDim.x * blockIdx.x + threadIdx.x;
	thatPoint = size;
	while(tid < size-1)
	{
		float firstFloat = aCopy[tid];
		float secondFloat = aCopy[tid+1];

		if(firstFloat > 0 && secondFloat < 0)
		{
				thatPoint = tid; // Solo uno puede entrar aquí
				*cuttingIdx = tid;
		}

		tid += blockDim.x*gridDim.x;
	}
	
	__syncthreads();
	
	//__shared__ float cachePos[1024];
	//__shared__ float sumNegShared;
	__shared__ bool breakFlux;
	breakFlux = false;

	sumNegShared = *negSum;

	// En un proceso iterativo y redux obtengo el indice de corte.
	int valuesToSum = *cuttingIdx;
	int miniBlockSize = (valuesToSum+1)/blockDim.x+1;

	int finBlock = valuesToSum - miniBlockSize*threadIdx.x;
	int iniBlock = valuesToSum - miniBlockSize*(threadIdx.x+1);

	float parSum = 0.0;

	for(int indice = finBlock; indice > iniBlock && ! breakFlux; indice--)
	{
		if(indice < 0) continue;
		parSum += a[ind[indice]];

		// Quizas no deberia estar aqui
		if(parSum > -sumNegShared && threadIdx.x == 0)
		{
			*cuttingIdx = indice;
			breakFlux = true;
		}
	}

	// Si ya hemos acabado, volvemos. No es lo normal, quizas conviene quitarlo
	// para ir mas rapido en la mayoria de los casos.
	__syncthreads();
	if(breakFlux) return;

	// Esto podria sobrar
	if(threadIdx.x == 0) *posSum = parSum;
	
	// Almacenamos las sumas parciales en el array de cache
	// Ordenamos el array a la inversa... asi funciona todo
	// el algoritmo que teniamos ya en fucnionamiento.
	//int cacheIndex = threadIdx.x;
	cachePos[cacheIndex] = parSum;

	__shared__ float sumAccumulated; 
	__shared__ int idxAccumulated;
	__shared__ bool refined;
	__shared__ bool cutIdx;

	idxAccumulated = 0;
	sumAccumulated = 0.0;

	refined = false;
	cutIdx = false;
	float tempSum = 0.0;
	int restValuesToSum = valuesToSum;

	while(!refined)
	{
		// inicializacion de cache.
		int threadReposition = threadIdx.x-idxAccumulated;
		cachePos[threadIdx.x] = 0.0;
		if(threadReposition >= 0)
		{
			cachePos[threadReposition] = parSum;
		}
		__syncthreads();

		cutIdx = false;
		int offset = 1;
		int particion = offset*2;
		while(!cutIdx && offset <= restValuesToSum)
		{
			int secondTempIdx = threadReposition+offset;
			if(threadReposition >= 0 &&  threadReposition%particion == 0)
			{
				tempSum = cachePos[threadReposition] + cachePos[secondTempIdx];

				if(threadReposition == 0 && tempSum + sumAccumulated > -sumNegShared)
				{
					cutIdx = true;
					sumAccumulated += cachePos[threadReposition];
					idxAccumulated += offset;
				}
				else
				{
					cachePos[threadReposition] = tempSum;	
				}
			}

			__syncthreads();
			
			offset = particion;
			particion = offset*2;
		}

		__syncthreads();

		restValuesToSum = valuesToSum - idxAccumulated;

		if(threadReposition == 0 && (sumAccumulated > -sumNegShared || restValuesToSum <= 0))
		{
			refined = true;
		}

	}

	__syncthreads();

	if(cacheIndex == 0)
	{
		*posSum = sumAccumulated;
		*cuttingIdx = *cuttingIdx - idxAccumulated*miniBlockSize;
	}		
}
*/

__global__ void sort(float *a, float *aCopy, int* ind, float* negSum, int* lock, int size, int* cuttingIdx, float *posSum)
{
	int elem = threadIdx.x + blockDim.x * blockIdx.x;
	while(elem < size)
	{
		aCopy[elem] = a[elem];
		ind[elem] = elem;

		elem = elem + blockDim.x * gridDim.x;
	}

	__syncthreads();

	float cacheFirst = 0;
	float cacheSecond = 0;
	float cacheThird = 0;

	int cacheFirstInd;
	int cacheSecondInd;
	int cacheThirdInd ;

	for (int j = 0; j < size/2+1 ; j++) 
	{
		int i = (threadIdx.x + blockDim.x * blockIdx.x)*2;
		
		while(i+1 < size)
		{
			cacheFirst = aCopy[i];
			cacheSecond = aCopy[i+1];

			cacheFirstInd = ind[i];
			cacheSecondInd = ind[i+1];

			if(cacheFirst < cacheSecond) 
			{
				// Weight
				float temp = cacheFirst;
				aCopy[i] = cacheSecond;
				aCopy[i+1] = temp;

				// Indirection
				int tempInd = cacheFirstInd;
				ind[i] = cacheSecondInd;
				ind[i+1] = tempInd;
			}

			i += (blockDim.x*gridDim.x)*2;
		}

		__syncthreads();

		i = (threadIdx.x + blockDim.x*blockIdx.x)*2 +1;
	
		while(i+1 < size)
		{
			cacheFirst = aCopy[i];
			cacheSecond = aCopy[i+1];

			cacheFirstInd = ind[i];
			cacheSecondInd = ind[i+1];

			if(cacheFirst < cacheSecond) 
			{
				// Weight
				float temp = cacheFirst;
				aCopy[i] = cacheSecond;
				aCopy[i+1] = temp;

				// Indirection
				int tempInd = cacheFirstInd;
				ind[i] = cacheSecondInd;
				ind[i+1] = tempInd;
			}
			i += (blockDim.x*gridDim.x)*2;
		}
	}

	__syncthreads();

	__shared__ int thatPoint;
	__shared__ float cacheNeg[threadsPerOnlyOneBlock];
	__shared__ float sumNegShared;
    
	sumNegShared = 0.0;
	
	int i = blockDim.x * blockIdx.x + threadIdx.x;
	int tid = blockDim.x * blockIdx.x + threadIdx.x;
	int cacheIndex = threadIdx.x;
	
	float tempNeg = 0.0;
	float tempPos = 0.0;
	while(tid < size)
	{
		float aCopyTemp = aCopy[tid];
		if(aCopyTemp < 0.0)
			tempNeg += aCopyTemp;

		tid += blockDim.x*gridDim.x;
	}

    cacheNeg[cacheIndex] = tempNeg;

	__syncthreads();

	int NewI = blockDim.x/2;
	while(NewI!=0)
	{
		if(cacheIndex < NewI)
		{
			cacheNeg[cacheIndex] += cacheNeg[cacheIndex+NewI];
		}

		__syncthreads();
		NewI/=2;
	}

	if(cacheIndex == 0)
	{
		*negSum = cacheNeg[0];
		sumNegShared = cacheNeg[0];
	}
	
	__syncthreads();

	
	// Buscar el primer indice donde queda anulada la suma negativa.
	tid = blockDim.x * blockIdx.x + threadIdx.x;
	thatPoint = size;
	while(tid < size-1)
	{
		float firstFloat = aCopy[tid];
		float secondFloat = aCopy[tid+1];

		if(firstFloat > 0 && secondFloat < 0)
		{
				thatPoint = tid; // Solo uno puede entrar aquí
				*cuttingIdx = tid;
		}

		tid += blockDim.x*gridDim.x;
	}
}
__global__ void getBalancedThreshold(float *a, int* ind, float* negSum, int size, int* cuttingIdx, float *posSum)
{
	__shared__ float cachePos[threadsPerOnlyOneBlock];
	__shared__ float sumNegShared;
	__shared__ bool breakFlux;
	breakFlux = false;

	sumNegShared = *negSum;

	// En un proceso iterativo y redux obtengo el indice de corte.
	int valuesToSum = *cuttingIdx;
	int miniBlockSize = (valuesToSum+1)/blockDim.x+1;

	int finBlock = valuesToSum - miniBlockSize*threadIdx.x;
	int iniBlock = valuesToSum - miniBlockSize*(threadIdx.x+1);

	float parSum = 0.0;

	for(int indice = finBlock; indice > iniBlock && ! breakFlux; indice--)
	{
		if(indice < 0) continue;
		parSum += a[ind[indice]];

		// Quizas no deberia estar aqui
		if(parSum > -sumNegShared && threadIdx.x == 0)
		{
			*cuttingIdx = indice;
			breakFlux = true;
		}
	}

	// Si ya hemos acabado, volvemos. No es lo normal, quizas conviene quitarlo
	// para ir mas rapido en la mayoria de los casos.
	__syncthreads();
	if(breakFlux) return;

	// Esto podria sobrar
	if(threadIdx.x == 0) *posSum = parSum;
	
	// Almacenamos las sumas parciales en el array de cache
	// Ordenamos el array a la inversa... asi funciona todo
	// el algoritmo que teniamos ya en fucnionamiento.
	int cacheIndex = threadIdx.x;
	cachePos[cacheIndex] = parSum;

	__shared__ float sumAccumulated; 
	__shared__ int idxAccumulated;
	__shared__ bool refined;
	__shared__ bool cutIdx;

	idxAccumulated = 0;
	sumAccumulated = 0.0;

	refined = false;
	cutIdx = false;
	float tempSum = 0.0;
	int restValuesToSum = valuesToSum;

	while(!refined)
	{
		// inicializacion de cache.
		int threadReposition = threadIdx.x-idxAccumulated;
		cachePos[threadIdx.x] = 0.0;
		if(threadReposition >= 0)
		{
			cachePos[threadReposition] = parSum;
		}
		__syncthreads();

		cutIdx = false;
		int offset = 1;
		int particion = offset*2;
		while(!cutIdx && offset <= restValuesToSum)
		{
			int secondTempIdx = threadReposition+offset;
			if(threadReposition >= 0 &&  threadReposition%particion == 0)
			{
				tempSum = cachePos[threadReposition] + cachePos[secondTempIdx];

				if(threadReposition == 0 && tempSum + sumAccumulated > -sumNegShared)
				{
					cutIdx = true;
					sumAccumulated += cachePos[threadReposition];
					idxAccumulated += offset;
				}
				else
				{
					cachePos[threadReposition] = tempSum;	
				}
			}

			__syncthreads();
			
			offset = particion;
			particion = offset*2;
		}

		__syncthreads();

		restValuesToSum = valuesToSum - idxAccumulated;

		if(threadReposition == 0 && (sumAccumulated > -sumNegShared || restValuesToSum <= 0))
		{
			refined = true;
		}

	}

	__syncthreads();

	if(cacheIndex == 0)
	{
		*posSum = sumAccumulated;
		*cuttingIdx = *cuttingIdx - idxAccumulated*miniBlockSize;
	}
	
}

__global__ void swapOnKernel(int *a, int size)
{
    int i = blockDim.x * blockIdx.x + threadIdx.x * 2;
	int cacheFirst;
	int cacheSecond;
	int cacheThird;

    for (int j = 0; j < size/2 + 1; j++) 
	{
		i = blockDim.x * blockIdx.x + threadIdx.x * 2;
		while(i < size)
		{
			if(i+1 < size) {
				cacheFirst = a[i];
				cacheSecond = a[i+1];

				if(cacheFirst > cacheSecond) {
					int temp = cacheFirst;
					a[i] = cacheSecond;
					cacheSecond = a[i+1] = temp;
				}
			}

			__syncthreads();

			if(i+2 < size) {
				cacheThird = a[i+2];
				if(cacheSecond > cacheThird) {
					int temp = cacheSecond;
					a[i+1] = cacheThird;
					a[i+2] = temp;
				}
			}

			i += (blockDim.x * gridDim.x)*2;
		}

		__syncthreads();
    }

}

__host__ void bubbleSort(float arr[], int n) {

	bool swapped = true;
	int j = 0;
	float tmp;
	while (swapped) {
		swapped = false;
		j++;
		for (int i = 0; i < n - j; i++) {
			if (arr[i] > arr[i + 1]) {
				tmp = arr[i];
				arr[i] = arr[i + 1];
				arr[i + 1] = tmp;
				swapped = true;
			}
		}
	}
}

int sortAndGetThreashold(float* weights, int* indirection, int* threshold, int size, bool verbose)
{
	// Create timer
	cudaEvent_t start, stop;
	float time = 0.0;
	if(verbose)
	{
		cudaEventCreate(&start);
		cudaEventCreate(&stop);
		cudaEventRecord(start, 0);
	}

	float *dev_aCopy = 0;
    cudaError_t cudaStatus;

    // Choose which GPU to run on, change this on a multi-GPU system.
    cudaStatus = cudaSetDevice(0);
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaSetDevice failed!  Do you have a CUDA-capable GPU installed?");
        goto Error;
    }

	// Allocate GPU buffers for one vectors.
    cudaStatus = cudaMalloc((void**)&dev_aCopy, size * sizeof(float));
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMalloc failed!");
        goto Error;
    }

    // Launch a kernel on the GPU with one thread for each element.
	
	int maxThreads = threadsPerBlock;

	int computingThreads = size/2;
	if(computingThreads > maxThreads)
		computingThreads = maxThreads;

	float* negSum;
	float* posSum;
	int* lock;

	cudaStatus = cudaMalloc((void**)&negSum, sizeof(float));
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMalloc negSum failed!");
        goto Error;
    }

	cudaStatus = cudaMalloc((void**)&posSum, sizeof(float));
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMalloc posSum failed!");
        goto Error;
    }

	cudaStatus = cudaMalloc((void**)&lock, sizeof(int));
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMalloc lock failed!");
        goto Error;
    }

	int hostLock = 0;
    cudaStatus = cudaMemcpy(lock, &hostLock, sizeof(int), cudaMemcpyHostToDevice);
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMemcpy hostLock failed!");
        goto Error;
    }

	// Realizamos la ordenacion del array de pesos y lo codificamos en dev_ind
	sort<<<1, computingThreads>>>(weights, dev_aCopy, indirection, negSum, lock, size, threshold , posSum);

	if(verbose)
	{
		cudaStatus = cudaDeviceSynchronize();
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "sort fallo: %s\n", cudaGetErrorString(cudaStatus));
			fflush(0);
			return cudaStatus;
		}
	}

	// Obtenemos el indice de corte
	getBalancedThreshold<<<1, computingThreads>>>(weights, indirection, negSum, size, threshold, posSum);

	if(verbose)
	{
		cudaEventRecord(stop, 0);
		cudaEventSynchronize(stop);
		cudaEventElapsedTime(&time, start, stop);

		cudaStatus = cudaDeviceSynchronize();
		if (cudaStatus != cudaSuccess) {
			printf("cudaDeviceSynchronize returned error code %d after launching addKernel!\n", cudaStatus);
			printf("getBalancedThreshold failed: %s\n", cudaGetErrorString(cudaStatus));fflush(0);
			goto Error;
		}
		else
			printf("ordenacion correcta!\n");

		float *a = (float*)malloc(size * sizeof(float));
		float *a2 = (float*)malloc(size * sizeof(float));
		// Copy output vector from GPU buffer to host memory.
		cudaStatus = cudaMemcpy(a, weights, size * sizeof(float), cudaMemcpyDeviceToHost);
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "cudaMemcpy aCopy failed!");
			goto Error;
		}

		// Copy output vector from GPU buffer to host memory.
		cudaStatus = cudaMemcpy(a2, dev_aCopy, size * sizeof(float), cudaMemcpyDeviceToHost);
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "cudaMemcpy aCopy failed!");
			goto Error;
		}

		int *ind = (int*)malloc(size * sizeof(int));
		cudaStatus = cudaMemcpy(ind, indirection, size * sizeof(int), cudaMemcpyDeviceToHost);
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "cudaMemcpy ind failed!");
			goto Error;
		}

		int cuttingIdxHost2 = 0;
		cudaStatus = cudaMemcpy(&cuttingIdxHost2, threshold, sizeof(int), cudaMemcpyDeviceToHost);
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "cudaMemcpy cuttingIdxHost failed!");
			goto Error;
		}

		printf("\n\nCUDA RESULTS...\n");
		printf("Hemos cortado en :%d\n", cuttingIdxHost2);
		float sumNegHost = 0;
		float sumPosHost = 0;
		cudaStatus = cudaMemcpy(&sumNegHost, negSum, sizeof(float), cudaMemcpyDeviceToHost);
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "cudaMemcpy ind failed!");
			goto Error;
		}
		cudaStatus = cudaMemcpy(&sumPosHost, posSum, sizeof(float), cudaMemcpyDeviceToHost);
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "cudaMemcpy ind failed!");
			goto Error;
		}
		printf("Sumatorio total negativo: %f\n sumatorio de corte: %f\n", sumNegHost, sumPosHost);

		double sumaTotal = 0.0;
		double sumaPos = 0.0;
		double sumaNeg = 0.0;
		double sumaTh = 0.0;
		double sumaPreTh = 0.0;
		int threshold = -1;
		double sumaAcumuladaTh = 0.0;
		double ordenado = 99999999;

		double restoPos = 0.0;
		double relacionSumatorioTh = 0.0;

		int change = 0;
		printf("\n\nCPU RESULTS...\n");
		for(int i = 0; i < size; i++)
		{
			float value = a[ind[i]];

			if(a2[i] != value)
			{
				printf("PROBLEMA!!! -> La indireccion no esta bien,\n");
				printf(" o hay una incongruencia con los vectores ordenados: %d -> [%f][%f]\n", i, a2[i], value);
			}

			if(value < 0 && ordenado >= 0)
				printf("pos->neg: %d\n", i);

			if(value > ordenado)
				printf("No esta ordenado!: %d -> [%f][%f]\n", i, a2[i], value);

			ordenado = value;

			sumaTotal += value;
			if(value >= 0)
			{
				sumaPos += value;
			}
			else sumaNeg += value;

			if(i >= cuttingIdxHost2 && value >= 0)
				relacionSumatorioTh += value;

			else if(value >= 0)
				sumaTh += value;
		}

		for(int i = size-1; i>=0; i--)
		{
			float value = a[ind[i]];
			if(value >= 0)
			{
				if(threshold < 0)
				{
					sumaAcumuladaTh += value;

					if(sumaAcumuladaTh > -sumaNeg)
						threshold = i;
				}
				else
				{
					restoPos += value;
				}
			}
		}

		printf("Suma total: %f\n", sumaTotal);
		printf("sumaNeg total: %f\n", sumaNeg);
		printf("resto de posSum: %f\n", restoPos);

		printf("sumaAcumuladaTh: %f\n", sumaAcumuladaTh);

		printf("relacion sumaAcumuladaTh con cuda: %f\n", relacionSumatorioTh);
		printf("threshold: %d\n", threshold);
	}


Error:
	// Solo eliminamos datos temporales...
	cudaFree(dev_aCopy);
	cudaFree(negSum);
	cudaFree(posSum);

    return cudaStatus;
}

int thrust_sort(void)
{
    // H has storage for 4 integers
    thrust::host_vector<int> H(4);

    // initialize individual elements
    H[0] = 14;
    H[1] = 20;
    H[2] = 38;
    H[3] = 46;
    
    // H.size() returns the size of vector H
    std::cout << "H has size " << H.size() << std::endl;

    // print contents of H
    for(int i = 0; i < H.size(); i++)
        std::cout << "H[" << i << "] = " << H[i] << std::endl;

    // resize H
    H.resize(2);
    
    std::cout << "H now has size " << H.size() << std::endl;

    // Copy host_vector H to device_vector D
    thrust::device_vector<int> D = H;
    
    // elements of D can be modified
    D[0] = 99;
    D[1] = 88;
    
    // print contents of D
    for(int i = 0; i < D.size(); i++)
        std::cout << "D[" << i << "] = " << D[i] << std::endl;

    // H and D are automatically deleted when the function returns
    return 0;
}

int doSorting(int number)
{
	srand((unsigned)time(0)); 

	int arraySize;

	if(number > 10) arraySize = number;
	else arraySize = 10;

	// Create vector and fill it with values
	thrust::host_vector<float> Ahost(arraySize);
	thrust::host_vector<int> indhost(arraySize);
	std::vector<float> a(arraySize);
	for (int i = 0; i < arraySize; i++) 
	{
		a[i] = 0.6-((float)rand()/RAND_MAX );
		Ahost[i] = a[i];
		indhost[i] = i;
	}
	std::vector<float> b(a);

	thrust::device_vector<float> A = Ahost;
	thrust::device_vector<int> ind = indhost;

	int64 stlSortStart1 = GetTimeMs64();
	thrust::sort_by_key(A.begin(), A.end(), ind.begin(), thrust::greater<float>());
	int64 stlSortFinish1 = GetTimeMs64();

	thrust::copy(ind.begin(), ind.end(), indhost.begin());

	float time = 0.0;
    // Swap elements in parallel.
    //cudaError_t cudaStatus = sortWithCuda(&a[0], &ind[0], a.size(), &time);

	//if (cudaStatus != cudaSuccess) {
    //    fprintf(stderr, "sortWithCuda failed!");
    //    return 1;
    //}

	int64 stlSortStart = GetTimeMs64();
	bubbleSort(&b[0], b.size());
	int64 stlSortFinish = GetTimeMs64();
	FILE* fout;
	fout = fopen("C:\\Users\\chus\\Documents\\dev\\Data\\models\\vel.txt", "a");

	fprintf (fout, " %d, %d, ", arraySize, stlSortFinish1 - stlSortStart1);
	fprintf (fout, "%d, ", (stlSortFinish - stlSortStart));

	bool sortingSuccessful = true;
	for (int i = 0; i < Ahost.size()-1  /*&& sortingSuccessful*/; i++) 
	{
		//printf("Valores: posicion %d---> %d %f %f\n ", i, indhost[i], Ahost[indhost[i]], b[Ahost.size()-1-i]);
		if (Ahost[indhost[i]] < Ahost[indhost[i+1]]) 
		{
			sortingSuccessful = false;
			printf("esta desordenado: posicion %d---> %f %f\n ", i, Ahost[indhost[i]], Ahost[indhost[i+1]]);
		}
		if(Ahost[indhost[i]] != b[Ahost.size()-1-i])
		{
			sortingSuccessful = false;
			printf("No es igual: posicion %d---> %f %f\n ", i, Ahost[indhost[i]], b[Ahost.size()-1-i]);
		}
	}
	if(!sortingSuccessful) {
		printf("Sorting failed.\n");
	}

	fprintf(fout, " %f\n", ((double)(stlSortFinish - stlSortStart))/(double)(stlSortFinish1 - stlSortStart1));
	fclose(fout);

	Ahost.clear();
	A.clear();
	Ahost.shrink_to_fit();
	A.shrink_to_fit();
	ind.clear();
	ind.shrink_to_fit();
	indhost.clear();
	indhost.shrink_to_fit();

    // cudaDeviceReset must be called before exiting in order for profiling and
    // tracing tools such as Parallel Nsight and Visual Profiler to show complete traces.
    cudaError_t cudaStatus = cudaDeviceReset();
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaDeviceReset failed!");
        return 1;
    }

	//getchar();

    return 0;
}

// Helper function for using CUDA to sort vectors in parallel.
__host__ cudaError_t sortWithCuda(float *a, int *ind, size_t size, float* time)
{
    float *dev_a = 0;
	float *dev_aCopy = 0;
	int *dev_ind = 0;
    cudaError_t cudaStatus;

    // Choose which GPU to run on, change this on a multi-GPU system.
    cudaStatus = cudaSetDevice(0);
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaSetDevice failed!  Do you have a CUDA-capable GPU installed?");
        goto Error;
    }

    // Allocate GPU buffers for one vectors.
    cudaStatus = cudaMalloc((void**)&dev_a, size * sizeof(float));
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMalloc failed!");
        goto Error;
    }

	// Allocate GPU buffers for one vectors.
    cudaStatus = cudaMalloc((void**)&dev_aCopy, size * sizeof(float));
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMalloc failed!");
        goto Error;
    }

    cudaStatus = cudaMalloc((void**)&dev_ind, size * sizeof(int));
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMalloc failed!");
        goto Error;
    }

     // Copy input vectors from host memory to GPU buffers.
    cudaStatus = cudaMemcpy(dev_a, a, size * sizeof(float), cudaMemcpyHostToDevice);
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMemcpy failed!");
        goto Error;
    }

    // Launch a kernel on the GPU with one thread for each element.
	
	int maxThreads = threadsPerOnlyOneBlock;

	int computingThreads = size/2;
	if(computingThreads > maxThreads)
		computingThreads = maxThreads;

	int* cutting_idx;
	cudaStatus = cudaMalloc((void**)&cutting_idx, sizeof(int));
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMalloc cutting_idx failed!");
        goto Error;
    }

	float* negSum;
	cudaStatus = cudaMalloc((void**)&negSum, sizeof(float));
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMalloc negSum failed!");
        goto Error;
    }

	float* posSum;
	cudaStatus = cudaMalloc((void**)&posSum, sizeof(float));
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMalloc posSum failed!");
        goto Error;
    }

	int* lock;
	cudaStatus = cudaMalloc((void**)&lock, sizeof(int));
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMalloc lock failed!");
        goto Error;
    }

	int hostLock = 0;
    cudaStatus = cudaMemcpy(lock, &hostLock, sizeof(int), cudaMemcpyHostToDevice);
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMemcpy hostLock failed!");
        goto Error;
    }


	// Create timer
	cudaEvent_t start, stop;
	cudaEventCreate(&start);
	cudaEventCreate(&stop);
	cudaEventRecord(start, 0);
	
	sort<<<1, computingThreads>>>(dev_a, dev_aCopy, dev_ind, negSum, lock, size, cutting_idx , posSum);

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "sort fallo: %s\n", cudaGetErrorString(cudaStatus));
		fflush(0);
		return cudaStatus;
	}

	int cuttingIdxHost = 0;
	cudaStatus = cudaMemcpy(&cuttingIdxHost, cutting_idx, sizeof(int), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemcpy cuttingIdxHost failed!");
		goto Error;
	}

	getBalancedThreshold<<<1, computingThreads>>>(dev_a, dev_ind, negSum, size, cutting_idx, posSum);

    cudaEventRecord(stop, 0);
	cudaEventSynchronize(stop);
	cudaEventElapsedTime(time, start, stop);

	cudaStatus = cudaDeviceSynchronize();
    if (cudaStatus != cudaSuccess) {
        printf("cudaDeviceSynchronize returned error code %d after launching addKernel!\n", cudaStatus);
        printf("getBalancedThreshold failed: %s\n", cudaGetErrorString(cudaStatus));fflush(0);
		goto Error;
    }
	else
		printf("getBalancedThreshold correcta!\n");

	float *a2 = (float*)malloc(size * sizeof(float));
    // Copy output vector from GPU buffer to host memory.
    cudaStatus = cudaMemcpy(a, dev_a, size * sizeof(float), cudaMemcpyDeviceToHost);
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMemcpy aCopy failed!");
        goto Error;
    }

	// Copy output vector from GPU buffer to host memory.
    cudaStatus = cudaMemcpy(a2, dev_aCopy, size * sizeof(float), cudaMemcpyDeviceToHost);
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMemcpy aCopy failed!");
        goto Error;
    }

    cudaStatus = cudaMemcpy(ind, dev_ind, size * sizeof(int), cudaMemcpyDeviceToHost);
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMemcpy ind failed!");
        goto Error;
    }

	int cuttingIdxHost2 = 0;
    cudaStatus = cudaMemcpy(&cuttingIdxHost2, cutting_idx, sizeof(int), cudaMemcpyDeviceToHost);
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMemcpy cuttingIdxHost failed!");
        goto Error;
    }

	printf("\n\nCUDA RESULTS...\n");
	printf("pos->neg :%d\n", cuttingIdxHost);
	printf("Hemos cortado en :%d\n", cuttingIdxHost2);
	float sumNegHost = 0;
	float sumPosHost = 0;
	cudaStatus = cudaMemcpy(&sumNegHost, negSum, sizeof(float), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemcpy ind failed!");
		goto Error;
	}
	cudaStatus = cudaMemcpy(&sumPosHost, posSum, sizeof(float), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemcpy ind failed!");
		goto Error;
	}
	printf("Sumatorio total negativo: %f\n sumatorio de corte: %f\n", sumNegHost, sumPosHost);

	double sumaTotal = 0.0;
	double sumaPos = 0.0;
	double sumaNeg = 0.0;
	double sumaTh = 0.0;
	double sumaPreTh = 0.0;
	int threshold = -1;
	double sumaAcumuladaTh = 0.0;
	double ordenado = 99999999;

	double restoPos = 0.0;
	double relacionSumatorioTh = 0.0;

	int change = 0;
	printf("\n\nCPU RESULTS...\n");
	for(int i = 0; i < size; i++)
	{
		float value = a[ind[i]];

		if(a2[i] != value)
			printf("PROBLEMA!!! -> La indireccion no esta bien o algo: %d -> [%f][%f]\n", i, a2[i], value);

		if(value < 0 && ordenado >= 0)
			printf("pos->neg: %d\n", i);

		if(value > ordenado)
			printf("No esta ordenado!: %d -> [%f][%f]\n", i, a2[i], value);

		ordenado = value;

		sumaTotal += value;
		if(value >= 0)
		{
			sumaPos += value;
		}
		else sumaNeg += value;

		if(i >= cuttingIdxHost2 && value >= 0)
			relacionSumatorioTh += value;

		else if(value >= 0)
			sumaTh += value;
	}

	for(int i = size-1; i>=0; i--)
	{
		float value = a[ind[i]];
		if(value >= 0)
		{
			if(threshold < 0)
			{
				sumaAcumuladaTh += value;

				if(sumaAcumuladaTh > -sumaNeg)
					threshold = i;
			}
			else
			{
				restoPos += value;
			}
		}
	}

	printf("Suma total: %f\n", sumaTotal);
	printf("sumaNeg total: %f\n", sumaNeg);
	printf("resto de posSum: %f\n", restoPos);

	printf("sumaAcumuladaTh: %f\n", sumaAcumuladaTh);

	printf("relacion sumaAcumuladaTh con cuda: %f\n", relacionSumatorioTh);
	printf("threshold: %d\n", threshold);

Error:
    cudaFree(dev_a);
	cudaFree(dev_aCopy);
	cudaFree(dev_ind);
	cudaFree(negSum);
	cudaFree(posSum);
	cudaFree(cutting_idx);
    
    return cudaStatus;
}
