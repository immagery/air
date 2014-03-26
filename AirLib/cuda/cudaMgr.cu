#include "cudaMgr.cuh"
#include "cudaMVC.cuh"

cudaError_t addWithCuda(int *c, const int *a, const int *b, unsigned int size);

__global__ void addKernel(int *c, const int *a, const int *b)
{
    int i = threadIdx.x;
    c[i] = a[i] + b[i];
}

void cudaManager::loadModels(cudaModel* in_models, int modelsCount, bool collectData)
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

	//checkCudaErrors( cudaSetDeviceFlags( cudaDeviceMapHost ) );

	if(modelsLoaded > 0)
	{
		// tenemos que copiarlos luego
		printf("Ya hay modelos cargados! - libera memoria antes de cargar mas.\n");
		return;
	}

	if(modelsCount <= 0)
	{
		// tenemos que copiarlos luego
		printf("No hay modelos que cargar!!.\n");
		return;
	}

	//models = (cudaModel*)malloc(cudaModel::getMemorySize()*modelsCount);
	// Ojo con la gestion de memoria, podría no funcionar.
	models = in_models;
	modelsLoaded = modelsCount;

    // Specify texture object parameters
    struct cudaTextureDesc texDesc;
    memset(&texDesc, 0, sizeof(texDesc));
	texDesc.readMode         = cudaReadModeElementType;

	for(int i = 0; i< modelsCount; i++)
	{
		//models[i].npts = in_models[i].npts;
		//ntri = modelo.nt;
		int npts = in_models[i].npts;
		int ntri = in_models[i].ntri;

		blocksPerGridPoints = (npts+threadsPerBlock-1)/threadsPerBlock;
		if(blocksPerGridPoints > 32) blocksPerGridPoints = 32;

		blocksPerGridFaces = (ntri+threadsPerBlock-1)/threadsPerBlock;
		if(blocksPerGridFaces > 32) blocksPerGridFaces = 32;

		checkCudaErrors( cudaMalloc( (void**)&models[i].cudaSum, sizeof(PRECISION) ) );

		int size = (npts*npts+npts)/2*sizeof(PRECISION);

		// Textures init for faster computations.
		checkCudaErrors( cudaMalloc( (void**)&models[i].cudaPositions_forTex, npts * sizeof(PRECISION) *3 ) );
		checkCudaErrors( cudaMalloc( (void**)&models[i].cudaTriangles_forTex, ntri * sizeof(int) *3 ) );
		checkCudaErrors( cudaMalloc( (void**)&models[i].cudaBHDistances_forTex, size ) );

		checkCudaErrors( cudaMemcpy(models[i].cudaPositions_forTex, models[i].hostPositions, npts * 3 * sizeof(PRECISION), cudaMemcpyHostToDevice) );
		checkCudaErrors( cudaMemcpy(models[i].cudaTriangles_forTex, models[i].hostTriangles, ntri * 3 * sizeof(int), cudaMemcpyHostToDevice) );
		checkCudaErrors( cudaMemcpy(models[i].cudaBHDistances_forTex, models[i].hostBHDistances, size , cudaMemcpyHostToDevice) );

		// int lockers for float computations
		int* hostLocks = (int*)malloc(sizeof(int)*npts);
		for(int hostIdx = 0; hostIdx < npts; hostIdx++) hostLocks[hostIdx] = 0;
		checkCudaErrors( cudaMalloc( (void**)&models[i].lockVertexes, npts * sizeof(int) ) );
		checkCudaErrors( cudaMemcpy(models[i].lockVertexes, hostLocks, npts * sizeof(int), cudaMemcpyHostToDevice) );
		

		cudaError_t cudaStatus = cudaGetLastError();
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "memory allocation for cudaPositions_forTex: %s\n", cudaGetErrorString(cudaStatus));
			fflush(0);
			return;
		}


		// Specify texture
		struct cudaResourceDesc resDesc;
		memset(&resDesc, 0, sizeof(resDesc));
		resDesc.resType = cudaResourceTypeLinear;
		resDesc.res.linear.devPtr = models[i].cudaPositions_forTex;
		resDesc.res.linear.desc.f = cudaChannelFormatKindFloat;
		resDesc.res.linear.desc.x = 32; // bits per channel
		resDesc.res.linear.sizeInBytes = npts*3*sizeof(PRECISION);

		// create texture object: we only have to do this once!
		cudaCreateTextureObject(&models[i].cudaPositionsTex, &resDesc, &texDesc, NULL);

		struct cudaResourceDesc resDescTri;
		memset(&resDescTri, 0, sizeof(resDescTri));
		resDescTri.resType = cudaResourceTypeLinear;
		resDescTri.res.linear.devPtr = models[i].cudaTriangles_forTex;
		resDescTri.res.linear.desc.f = cudaChannelFormatKindSigned;
		resDescTri.res.linear.desc.x = 32; // bits per channel
		resDescTri.res.linear.sizeInBytes = ntri*3*sizeof(int);

		cudaCreateTextureObject(&models[i].cudaTrianglesTex, &resDescTri, &texDesc, NULL);

		cudaStatus = cudaGetLastError();
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "crear texture para triangulos: %s\n", cudaGetErrorString(cudaStatus));
			fflush(0);
			return;
		}

		// Temp computations
		checkCudaErrors( cudaMalloc( (void**)&models[i].cudaUnitVectors, npts * sizeof(PRECISION3) ) );
		checkCudaErrors( cudaMalloc( (void**)&models[i].cudaNormas, npts * sizeof(PRECISION) ) );
		checkCudaErrors( cudaMalloc( (void**)&models[i].cudaWeights, ntri * sizeof(PRECISION) * 3 ) );
		checkCudaErrors( cudaMalloc( (void**)&models[i].resWeights, npts * sizeof(PRECISION) ) );
		

		models[i].cudaCoords = (PRECISION*)malloc(ntri*sizeof(PRECISION) * 3);

		/*if(collectData)
		{
			//checkCudaErrors( cudaMalloc( (void**)&cudaCoords, npts * sizeof(PRECISION) ) );
			//tempResults = (PRECISION*) malloc(blocksPerGrid*sizeof(PRECISION));
			checkCudaErrors( cudaHostAlloc( (void**)&models[i].resWeights, 
											ntri * sizeof(PRECISION) *3, 
											cudaHostAllocMapped) );
	
			for(int triIdx = 0; triIdx< ntri*3; triIdx++)
				models[i].resWeights[triIdx] = 0.0;

			checkCudaErrors(cudaHostGetDevicePointer(&models[i].cudaWeights, models[i].resWeights, 0));
		}
		else
		{*/
		
		//}
								
		cudaStatus = cudaGetLastError();
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "final para mvc: %s\n", cudaGetErrorString(cudaStatus));
			fflush(0);
			return;
		}
	}
	loaded = true;

}

void cudaManager::freeModels(bool collectData)
{	
	for(int i = 0; i< modelsLoaded; i++)
	{
		// Unbind textures
		//cudaUnbindTexture(models[i].cudaPositionsTex);
		//cudaUnbindTexture(models[i].cudaTrianglesTex);
		//cudaUnbindTexture(models[i].cudaBHDistancesTex);

		cudaDestroyTextureObject(models[i].cudaPositionsTex);

		// Free model data memory
		cudaFree(models[i].cudaPositions_forTex);
		cudaFree(models[i].cudaTriangles_forTex);
		cudaFree(models[i].cudaBHDistances_forTex);


		// device Memory
		//cudaFree(models[i].cudaPositions);
		//cudaFree(models[i].cudaTriangles);
		cudaFree(models[i].cudaUnitVectors);
		cudaFree(models[i].cudaNormas);

		// Pined-mapped memory
		//cudaFreeHost(tempResults);

		cudaFree(models[i].lockVertexes);

		if( collectData)
			cudaFreeHost(models[i].resWeights);
		else
			cudaFree(models[i].cudaWeights);
	}

	loaded = false;
}

PRECISION timeLapseCuda(clock_t clock1,clock_t clock2)
{
	PRECISION diffticks=clock2-clock1;
    PRECISION diffms=diffticks/CLOCKS_PER_SEC;
    return diffms;
}

// PREMISAS -> Hay que asegurar todo esto desde fuera
void cudaManager::cudaMVC(PRECISION3& point,  PRECISION* weights, int modelId, int nodeId, bool collectData)
{
	if(modelId < 0 || modelId >= modelsLoaded)
		return;

	cudaModel& m = models[modelId];

	// no estoy haciendo comprobaciones de indices, puede ser peligroso
	// Buscar si el nodo ya existe
	int nodeIdxFound = -1;
	for(int i = 0; i< m.nodes.size(); i++)
	{
		if(m.nodes[i]->nodeId == nodeId)
		{
			nodeIdxFound = i;
			break;
		}
	}

	// En caso de no existir lo cremos y reservamos memoria
	if(nodeIdxFound < 0)
	{
		cudaError_t cudaStatus = cudaGetLastError();
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "ver si hay error acumulado antes : %s\n", cudaGetErrorString(cudaStatus));
			fflush(0);
			return;
		}

		cudaNode* node = new cudaNode(m.npts);

		cudaStatus = cudaGetLastError();
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "error en la creacion del nodo : %s\n", cudaGetErrorString(cudaStatus));
			fflush(0);
			return;
		}
		nodeIdxFound = m.nodes.size();
		m.nodes.push_back(node);
		m.nodes.back()->nodeId = nodeId;
	}

	cudaNode* nodePtr = NULL;
	nodePtr = m.nodes[nodeIdxFound];

	int* lockVertexes;
	checkCudaErrors( cudaMalloc((void**)&lockVertexes, sizeof(int)*m.npts ) );

	// Compute unit vectors and norms
	unitVectorsComputation<<< blocksPerGridPoints, threadsPerBlock >>>(m.npts, 
																	   point, 
																	   m.cudaPositionsTex, 
																	   m.cudaUnitVectors, 
																	   m.cudaNormas,
																	   nodePtr->cudaWeights,
																	   lockVertexes);
	checkCudaErrors( cudaThreadSynchronize() );

	cudaError_t cudaStatus = cudaGetLastError();
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "unitVectorsComputation fallo: %s\n", cudaGetErrorString(cudaStatus));
        fflush(0);
		return;
    }

	/*
	PRECISION* unitVecHost = (PRECISION*) malloc(m.npts*sizeof(PRECISION)*3);
	checkCudaErrors( cudaMemcpy(unitVecHost, m.cudaUnitVectors, m.npts*sizeof(PRECISION)*3, cudaMemcpyDeviceToHost) );
	PRECISION* normasHost = (PRECISION*) malloc(m.npts*sizeof(PRECISION));
	checkCudaErrors( cudaMemcpy(normasHost, m.cudaNormas, m.npts*sizeof(PRECISION), cudaMemcpyDeviceToHost) );

	
	FILE* fout;
	fout = fopen("C:\\Users\\chus\\Documents\\dev\\Data\\models\\tempData\\CUDAoutUnitNorms.txt", "w");

	fprintf(fout, "%d points...with point [%f, %f, %f]\n", m.npts, point.x, point.y, point.z); fflush(fout);
	for(int idxPts = 0; idxPts < m.npts; idxPts++)
	{
		fprintf(fout, "%d: [%f, %f, %f] -> [%5.10f, %5.10f, %5.10f], [%5.10f]\n", idxPts, 
			m.hostPositions[idxPts*3], m.hostPositions[idxPts*3+1],m.hostPositions[idxPts*3+2],
			unitVecHost[idxPts*3], unitVecHost[idxPts*3+1], unitVecHost[idxPts*3+2], normasHost[idxPts]); fflush(fout);
	}

	fclose(fout);
	*/

	// Specify texture object parameters
    struct cudaTextureDesc texDesc;
    memset(&texDesc, 0, sizeof(texDesc));
	texDesc.readMode = cudaReadModeElementType;

	// Specify texture
	struct cudaResourceDesc resDesc;
	memset(&resDesc, 0, sizeof(resDesc));
	resDesc.resType = cudaResourceTypeLinear;
	resDesc.res.linear.devPtr = m.cudaNormas;
	resDesc.res.linear.desc.f = cudaChannelFormatKindFloat;
	resDesc.res.linear.desc.x = 32; // bits per channel
	resDesc.res.linear.sizeInBytes = m.npts*sizeof(PRECISION);

	// create texture object: we only have to do this once!
	cudaCreateTextureObject(&m.cudaNormasTex, &resDesc, &texDesc, NULL);

	// Specify texture
	struct cudaResourceDesc resDesc2;
	memset(&resDesc2, 0, sizeof(resDesc2));
	resDesc2.resType = cudaResourceTypeLinear;
	resDesc2.res.linear.devPtr = m.cudaUnitVectors;
	resDesc2.res.linear.desc.f = cudaChannelFormatKindFloat;
	resDesc2.res.linear.desc.x = 32; // bits per channel
	resDesc2.res.linear.sizeInBytes = m.npts*3*sizeof(PRECISION);

	// create texture object: we only have to do this once!
	cudaCreateTextureObject(&m.cudaUnitVectorsTex, &resDesc2, &texDesc, NULL);

	// Coords computation
	coordsComputation<<< blocksPerGridFaces, threadsPerBlock >>>(m.ntri, 
																point, 
																m.cudaTrianglesTex, 
																m.cudaUnitVectorsTex, 
																m.cudaNormasTex,
																lockVertexes,
																nodePtr->cudaWeights);


	// destuimos los objetos de textura porque ya no son necesarios
	cudaDestroyTextureObject(m.cudaUnitVectorsTex);
	cudaDestroyTextureObject(m.cudaNormasTex);

	// Sincronización de threads para continuar
	checkCudaErrors( cudaThreadSynchronize() );
	cudaFree(lockVertexes);

	cudaStatus = cudaGetLastError();
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "coordsComputation failed: %s\n", cudaGetErrorString(cudaStatus));
        fflush(0);
		return;
    }

	/*
	PRECISION* weightsTempHost = (PRECISION*) malloc(m.npts*sizeof(PRECISION));
	checkCudaErrors( cudaMemcpy(weightsTempHost, nodePtr->cudaWeights, m.npts*sizeof(PRECISION), cudaMemcpyDeviceToHost) );

	fout = fopen("C:\\Users\\chus\\Documents\\dev\\Data\\models\\tempData\\CUDA_weights_sin_sumar.txt", "w");

	fprintf(fout, "%d points...with point [%f, %f, %f]\n", m.npts, point.x, point.y, point.z); fflush(fout);
	for(int idxPts = 0; idxPts < m.npts; idxPts++)
	{
		fprintf(fout, "%d:-> [%5.10f]\n", idxPts, 
			weightsTempHost[idxPts]); fflush(fout);
	}

	fclose(fout);
	*/

	int* lock;
	int hostLock = 0;
	checkCudaErrors( cudaMalloc((void**)&lock, sizeof(int) ) );
	checkCudaErrors( cudaMemcpy(lock, &hostLock, sizeof(int), cudaMemcpyHostToDevice) );
	
	PRECISION* cudaSum;
	PRECISION sumParcHost = 0;
	checkCudaErrors( cudaMalloc((void**)&cudaSum, sizeof(PRECISION)) );
	checkCudaErrors( cudaMemcpy(cudaSum, &sumParcHost, sizeof(PRECISION), cudaMemcpyHostToDevice) );

	sumWeightsRedux<<< blocksPerGridPoints, threadsPerBlock >>>(lock,
																nodePtr->cudaWeights, 
															    m.npts, 
															    cudaSum);


	checkCudaErrors( cudaThreadSynchronize() );
	cudaFree(lock);

	normalizeWeights<<< blocksPerGridPoints, threadsPerBlock >>>(nodePtr->cudaWeights, cudaSum, m.npts);
	checkCudaErrors( cudaThreadSynchronize() );
	cudaFree(cudaSum);
	

	//checkCudaErrors( cudaMemcpy(&sumParcHost, cudaSum, sizeof(PRECISION), cudaMemcpyDeviceToHost) );	
	//printf("Cuda sumaPesos : %5.10f\n", sumParcHost);

	cudaStatus = cudaGetLastError();
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "normalizeWeights failed: %s\n", cudaGetErrorString(cudaStatus));
        fflush(0);
		return;
    }
	
	//checkCudaErrors( cudaMemcpy(weights, nodePtr->cudaWeights, sizeof(PRECISION)*m.npts, cudaMemcpyDeviceToHost) );

	/*
	for(int i = 0; i< m.npts; i++)
	{
		weights[i] = weights[i]/sumParcHost;
	}
	*/
}

__global__ void preMult(double *in, int *ind, int indLength, double *matrix, int length, double *res)
{
	int tid = threadIdx.x + blockIdx.x * blockDim.x;
	while (tid < indLength)
	{
		int col = ind[tid];
		int offset = col*length;
		res[tid] = 0;
		
		int elemIdx = 0;
		while(elemIdx < indLength)
			res[tid] += in[ind[elemIdx]]*matrix[offset+ind[elemIdx]];

		tid += blockDim.x * gridDim.x;
	}
}

__global__ void dotIndirected(Lock lock, double *inA, int *ind, int indLength, double* inB, double *c)
{
	__shared__ PRECISION cache[threadsPerBlock];
	int tid = threadIdx.x + blockIdx.x * blockDim.x;
	int cacheIndex = threadIdx.x;

	PRECISION temp = 0;
	while (tid < indLength)
	{
		int idx = ind[tid];
		temp += inA[idx]*inB[idx];
		tid += blockDim.x * gridDim.x;
	}

	// set the cache values
	cache[cacheIndex] = temp;

	//Syncronize threads in this block
	__syncthreads();

	int i = blockDim.x/2;
	while (i != 0)
	{
		if(cacheIndex < i)
			cache[cacheIndex] += cache[cacheIndex + i];

		__syncthreads();
		i /=2;
	}

	if(cacheIndex == 0)
	{
		lock.lock();
		*c += cache[0];
		lock.unlock();
	}

}

double cudaManager::Precompute_Distances_CUDA(double* weights,
											  int indSize,
											  int* indirection,
											  int modelIdx,
											  int nodeIdx)
{
	// 1. buscar el nodo, porque a lo mejor los hemos calculado en desorden.
	int nodeFound = -1;
	for( int i = 0; i< models[modelIdx].nodes.size(); i++)
		if(models[modelIdx].nodes[i]->nodeId == nodeIdx)
			nodeFound = i;
	
	if(nodeFound < 0 || nodeFound >= models[modelIdx].nodes.size())
		return -1;

	cudaNode* node = models[modelIdx].nodes[nodeFound];

	/*
	// 2. lanzar el cálculo con el indice correcto.
	preMult<<< blocksPerGrid, threadsPerBlock >>>( models[modelIdx].cudaWeights[nodeIdx], 
												   models[modelIdx].cudaIndirection[nodeIdx], 
												   models[modelIdx].indirectionSize[nodeIdx], 
												   models[modelIdx].cudaBHDistancesTex)

	// 3. actualizar flags en CPU para saber si los datos son correctos.




	int size = npts;
	double res = 0;
	for(int j = 0; j< size; j++)
	{
		if(weights[indirection[j]] < threshold) 
			break;

		double sum = 0;
		for(int k = 0; k< size; k++)
		{
			double value = weights[indirection[k]]*BihDistances.get(indirection[j],indirection[k]);
			
			// Si el valor que tenemos que sumar es muy pequeno, lo despreciamos.
			if(weights[indirection[k]] < threshold) 
				break;

			sum += value;
		}

		res += sum*weights[indirection[j]];
	}

	return res;
	*/

	return 0;
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
