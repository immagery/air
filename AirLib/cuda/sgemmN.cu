// Written by Vasily Volkov.
// Copyright (c) 2008, The Regents of the University of California. 
// All rights reserved.

#include <time.h>

#include "sgemmN.cuh"
#include "cuda_runtime.h"

#define BLOCK_SIZE 32

__device__ void saxpy( float a, float *b, float *c )
{
    c[0] += a*b[0];
    c[1] += a*b[1];
    c[2] += a*b[2];
    c[3] += a*b[3];
    c[4] += a*b[4];
    c[5] += a*b[5];
    c[6] += a*b[6];
    c[7] += a*b[7];
    c[8] += a*b[8];
    c[9] += a*b[9];
    c[10] += a*b[10];
    c[11] += a*b[11];
    c[12] += a*b[12];
    c[13] += a*b[13];
    c[14] += a*b[14];
    c[15] += a*b[15];
}

__device__ void saxpy2( const float* a, const float *b, const float *a2, float *c )
{
    c[0]  += a[0] *b[0] * a2[0];
    c[1]  += a[1] *b[1] * a2[0];
    c[2]  += a[2] *b[2] * a2[0];
    c[3]  += a[3] *b[3] * a2[0];
    c[4]  += a[4] *b[4] * a2[0];
    c[5]  += a[5] *b[5] * a2[0];
    c[6]  += a[6] *b[6] * a2[0];
    c[7]  += a[7] *b[7] * a2[0];
    c[8]  += a[8] *b[8] * a2[0];
    c[9]  += a[9] *b[9] * a2[0];
    c[10] += a[10]*b[10] * a2[0];
    c[11] += a[11]*b[11] * a2[0];
    c[12] += a[12]*b[12] * a2[0];
    c[13] += a[13]*b[13] * a2[0];
    c[14] += a[14]*b[14] * a2[0];
    c[15] += a[15]*b[15] * a2[0];
}

__device__ void saxpy3( const float* a, const float *b, float *c )
{
    c[0]  += a[0] *b[0] ;
    c[1]  += a[1] *b[1] ;
    c[2]  += a[2] *b[2] ;
    c[3]  += a[3] *b[3] ;
    c[4]  += a[4] *b[4] ;
    c[5]  += a[5] *b[5] ;
    c[6]  += a[6] *b[6] ;
    c[7]  += a[7] *b[7] ;
    c[8]  += a[8] *b[8] ;
    c[9]  += a[9] *b[9] ;
    c[10] += a[10]*b[10] ;
    c[11] += a[11]*b[11] ;
    c[12] += a[12]*b[12] ;
    c[13] += a[13]*b[13] ;
    c[14] += a[14]*b[14] ;
    c[15] += a[15]*b[15] ;
}

__device__ void saxpy64( const float* a, const float *b, float *c )
{
	#pragma unroll
	for( int i = 0; i < 64; i++)
		c[i]  = a[i]*b[i] ;
}

__device__ void saxpy32( const float* a, const float *b, float *c, const float* balance )
{
	#pragma unroll
	for( int i = 0; i < 32; i++)
		c[i]  = a[i]*b[i]* balance[0];
}

__device__ void redux32sum( const float* a, float* res )
{
	float c2[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	float c[8] = {0,0,0,0,0,0,0,0};

	
	// 32 -> 16
	#pragma unroll
		for( int i = 0; i < 16; i++)
			c2[i]  = a[i] + a[16+i];

	// 16 -> 8
	#pragma unroll
		for( int i = 0; i < 8; i++)
			c[i] = c2[8+i] + c2[i];

	//8 -> 4
	#pragma unroll
		for( int i = 0; i < 4; i++)
			c2[i]  = c[4+i] + c[i];

	//4->2
	#pragma unroll
		for( int i = 0; i < 2; i++)
			c[i]  = c2[2+i] + c2[i];

	// 2->1
			*res = c[0] + c[1];
}

__device__ void redux64sum( const float* a, float* res )
{
	float c[32] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
				   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	float c2[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	// 64 -> 32
	#pragma unroll
		for( int i = 0; i < 64; i++)
			c[i]  = a[i] + a[32+i];

	
	// 32 -> 16
	#pragma unroll
		for( int i = 0; i < 16; i++)
			c2[i] = c[16+i] + c[i];

	// 16 -> 8
	#pragma unroll
		for( int i = 0; i < 8; i++)
			c[i] = c2[8+i] + c2[i];

	//8 -> 4
	#pragma unroll
		for( int i = 0; i < 4; i++)
			c2[i]  = c[4+i] + c[i];

	//4->2
	#pragma unroll
		for( int i = 0; i < 2; i++)
			c[i]  = c2[2+i] + c2[i];

	// 2->1
			*res = c[0] + c[1];
}
__device__ void redux16sum( const float* a, float* res )
{
	float c[8] = {0,0,0,0,0,0,0,0};

	// 16 -> 8
	#pragma unroll
		for( int i = 0; i < 8; i++)
			c[i]  += a[i] + a[i*2];

	//8 -> 4
	#pragma unroll
		for( int i = 0; i < 4; i++)
			c[i]  += a[i] + a[i*2];

	//4->2
	#pragma unroll
		for( int i = 0; i < 2; i++)
			c[i]  += a[i] + a[i*2];

	// 2->1
			*res = c[0] + c[1];
}

extern "C" __global__ void sgemmNT( const float *A, int lda, const float *B, int ldb, float* C, int ldc, int k, float alpha, float beta )
{
    const int inx = threadIdx.x;
    const int iny = threadIdx.y;

    const int ibx = blockIdx.x * 64;
    const int iby = blockIdx.y * 16;
    
	const int id = inx + iny*16;

    A += ibx + id;
    B += iby + inx + __mul24( iny, ldb );
    C += ibx + id  + __mul24( iby, ldc );
    const float *Blast = B + k*ldb;

    float c[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    __shared__ float bs[16][16];
    do
    {
#pragma unroll
        for( int i = 0; i < 16; i += 4 )
            bs[iny+i][inx]  = B[i*ldb];
        __syncthreads();

#pragma unroll
        for( int i = 0; i < 16; i++, A += lda )
            saxpy( A[0], &bs[i][0], c ); 

        B += 16*ldb;
        __syncthreads();
    } while( B < Blast );

    for( int i = 0; i < 16; i++, C += ldc )
        C[0] = alpha*c[i] + beta*C[0]; 
}	

extern "C" __global__ void sgemmNN( const float *A, int lda, const float *B, int ldb, float* C, int ldc, int k, float alpha, float beta )
{
    const int inx = threadIdx.x;
    const int iny = threadIdx.y;

    const int ibx = blockIdx.x * 64;
    const int iby = blockIdx.y * 16;
    
	const int id = inx + iny*16;

    A += ibx + id;
    B += inx + __mul24( iby + iny, ldb );
    C += ibx + id  + __mul24( iby, ldc );

    const float *Blast = B + k;

    float c[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    __shared__ float bs[16][17];
    do
    {
#pragma unroll
        for( int i = 0; i < 16; i += 4 )
            bs[inx][iny+i]  = B[i*ldb];
        __syncthreads();

#pragma unroll
        for( int i = 0; i < 16; i++, A += lda )
            saxpy( A[0], &bs[i][0], c ); 

        B += 16;
        __syncthreads();
    } while( B < Blast );

    for( int i = 0; i < 16; i++, C += ldc )
        C[0] = alpha*c[i] + beta*C[0]; 
}	


extern "C" __global__ void vec_mat_vec_mult(const float *A, int lda, 
											const float *B, int ldb, 
											float *C, int k, 
											float alpha, float beta )
{
	// FOR 16 threads

	
    const int inx = threadIdx.x; 
    //const int iny = threadIdx.y;

    const int ibx = blockIdx.x * 16;
    const int iby = blockIdx.y * 16;
    
	const int id = inx;

	const float* ARow = A;
	A += ibx + id;
	B += ibx;  

    const float *Blast = B + k;

	// vector de multiplicacion local
    float c[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	
	// vector de sumatorio a lo largo de la columna
	float r[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	
	// valor temporal
	float res = 0;
    
	// cargamos la parte del vector que toca
	__shared__ float bs[16];
	bs[id]  = A[0];
	C[id] = -1;

	__syncthreads();

	do
    {
        saxpy3( &B[0], &bs[0], c ); 
		redux16sum(c, &res);
		r[id] += res*ARow[0];

        __syncthreads();

		B += 16;
		ARow += 16;

    } while( B < Blast );

    redux16sum(r, &C[ibx]);
}	

extern "C" __global__ void vmSymv(const float *A, int lda, 
								  const float *B, int ldb, 
							      float *C, int k, int length, 
							      float alpha, float beta )
{
	// FOR 16 threads
    const int inx = threadIdx.x; 

    const int ibx = blockIdx.x * 16;
    const int iby = blockIdx.y * 16;
    
	const int id = threadIdx.x;

	const float* ARow = A;
	A += ibx + id;
	B += ibx + __mul24(iby+inx,ldb);  

    const float *Blast = ARow + k;

	// vector de multiplicacion local
    float c[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	
	// vector de sumatorio a lo largo de la columna
	__shared__ float r[16];
	
	// valor temporal
	float res = 0;
    
	// cargamos la parte del vector que toca
	__shared__ float bs[16];
	bs[id]  = A[0];
	r[id] = 0;
	__syncthreads();

	int steps = ldb/16;
	int counter = 0;

	do
    {
        saxpy3( &B[0], &bs[0], c ); 
		redux16sum(c, &res);
		r[id] += res*ARow[0];

        __syncthreads();

		B += 16 * ldb;
		ARow += 16;
		counter++;

    } while( counter < steps );

	if(id == 0)
		redux16sum(r, &C[ibx]);
}	


extern "C" __global__ void vmv(const float *A, int lda, 
							   const float *B, int ldb, 
							   float *C, int k, 
							   float alpha, float beta )
{
	// FOR 64 threads
    const int id = threadIdx.x;
	const int ibx = blockIdx.x * BLOCK_SIZE;

	B += ibx + id*ldb;

	// vector de multiplicacion local
    float c[BLOCK_SIZE] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
							0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	
	// cargamos la parte del vector que toca
	__shared__ float bs[BLOCK_SIZE];
	float r[BLOCK_SIZE] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
							0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	// valor temporal
	float res;
	bs[id]  = A[ibx + id];
	__syncthreads();

	const int BStep = ldb*BLOCK_SIZE;
	const float* blast = A + lda;

	do
    {
        saxpy32( &B[0], &bs[0], c, &A[id]); 
		redux32sum(c, &res);
		r[id] += res;

		B += BStep;
		A += BLOCK_SIZE;

    } while( A < blast );

	C[ibx+id] = r[id];
}	


extern "C" void ourSgemm( char transa, char transb, int m, int n, int k, float alpha, const float *A, int lda, const float *B, int ldb, float beta, float *C, int ldc )
{	
    dim3 grid( m/64, n/16 ), threads( 16, 4 );
    if( transb == 'N' || transb == 'n' )
        sgemmNN<<<grid, threads>>>( A, lda, B, ldb, C, ldc, k, alpha, beta );
    else
        sgemmNT<<<grid, threads>>>( A, lda, B, ldb, C, ldc, k, alpha, beta );
}	

//
//	auxiliary routines
//	
void fill( float *A, int n, int maxi )
{	
    for( int j = 0; j < n; j++ )
        A[j] = float( (rand()%(maxi*2+1)) - maxi ) / ( maxi + 1.f );
}	

float diff( int m, int n, float *A, int lda, float *B, int ldb )
{
    float err = 0;
    for( int j = 0; j < n; j++ )
        for( int i = 0; i < m; i++ )
            err = max( err, fabs( A[i+j*lda] - B[i+j*ldb] ) );
    return err;
}

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cuda_runtime.h>
#include "cublas_v2.h"
#define MSize 6
#define NSize 512*32
#define IDX2C(i,j,ld) (((j)*(ld))+(i))

static __inline__ void modify (cublasHandle_t handle, float *m, int ldm, int
								n, int p, int q, float alpha, float beta)
{
	cublasSscal (handle, n-p, &alpha, &m[IDX2C(p,q,ldm)], ldm);
	cublasSscal (handle, ldm-p, &beta, &m[IDX2C(p,q,ldm)], 1);
}

int cublas_example()
{

	cudaError_t cudaStat;
	cublasStatus_t stat;
	cublasHandle_t handle;
	int i, j;

    // Matriz simetrica, column-mayor
	float *B = (float*)malloc( (NSize*(NSize+1))/2*sizeof( float ) );
	float *BComplete = (float*)malloc( NSize*NSize*sizeof( float ) );

	// Vector
	float *AVec = (float*)malloc( NSize*sizeof( float ) );

	fill( BComplete, NSize*NSize, 31 );
	fill( AVec, NSize, 31 );

	int count = 0;
	for(int i = 0; i< NSize; i++)
	{
		for(int j = 0; j < NSize; j++)
		{
			if(j <= i)
			{
				BComplete[i*NSize + j] = BComplete[j * NSize + i];
				B[count] = BComplete[i*NSize + j];
				count++;
			}
		}
	}

	// resultados
	float *cuda_result = (float*)malloc( NSize*sizeof( float ) );

	for(int i = 0; i< NSize; i++)
		cuda_result[i]= 0;

	//float *cpu_result= (float*)malloc( NSize*sizeof( float ) );

	float* devPtrB, *devPtrAVec, *devPtrRes, devPtrBSym;
	cudaStat = cudaMalloc ((void**)&devPtrB, NSize*NSize*sizeof(float));
	cudaStat = cudaMalloc ((void**)&devPtrAVec, NSize*sizeof(float));
	cudaStat = cudaMalloc ((void**)&devPtrRes,  NSize*sizeof(float));

	cudaStat = cudaMalloc ((void**)&devPtrBSym, ((NSize+1)*NSize)/2*sizeof(float));


	if (cudaStat != cudaSuccess) {
		printf ("device memory allocation failed");
		return EXIT_FAILURE;
	}

	stat = cublasCreate(&handle);
	if (stat != CUBLAS_STATUS_SUCCESS) {
		printf ("CUBLAS initialization failed\n");
		return EXIT_FAILURE;
	}

	cublasSetAtomicsMode(handle, CUBLAS_ATOMICS_ALLOWED);

	stat = cublasSetMatrix (NSize, NSize, sizeof(float), BComplete, NSize, devPtrB, NSize);
	
	//cudaStat = cudaMemcpy(&devPtrBSym, &B, ((NSize+1)*NSize)/2*sizeof(float), cudaMemcpyHostToDevice);
	
	//cudaStat = cudaMemcpy(&devPtrB, &BComplete, NSize*NSize*sizeof(float), cudaMemcpyHostToDevice);

	stat = cublasSetVector (NSize, sizeof(float), AVec, 1, devPtrAVec, 1);
	stat = cublasSetVector (NSize, sizeof(float), cuda_result, 1, devPtrRes, 1);

	if (stat != CUBLAS_STATUS_SUCCESS) {
		printf ("data download failed");
		cudaFree (devPtrAVec);
		cublasDestroy(handle);
		return EXIT_FAILURE;
	}

	float alpha = 1;
	float beta = 0;

	cudaEvent_t start, end;
    Q( cudaEventCreate( &start ) );
    Q( cudaEventCreate( &end ) );
	cudaEventRecord(start, 0);
	
	float* sum = (float*)malloc(sizeof(float));

	stat = cublasSgemv( handle, CUBLAS_OP_T, NSize, NSize, &alpha, devPtrB, NSize, devPtrAVec, 1, &beta, devPtrRes, 1 );

	//stat = cublasSsymv( handle, CUBLAS_FILL_MODE_LOWER, NSize, &alpha, devPtrB, NSize, devPtrAVec, 1, &beta, devPtrRes, 1 );

	if (stat != CUBLAS_STATUS_SUCCESS) {
		printf ("cublasSgemv failed");
		cudaFree (devPtrAVec);
		cublasDestroy(handle);
		return EXIT_FAILURE;
	}

	cudaEventRecord(end, 0);
	cudaEventSynchronize(end);
	float time;
	cudaEventElapsedTime(&time, start, end);
	cudaEventRecord(start, 0);
	
	stat = cublasSdot( handle, NSize, devPtrAVec, 1, devPtrRes, 1, sum );

	cudaEventRecord(end, 0);
	cudaEventSynchronize(end);
	float time2;
	cudaEventElapsedTime(&time2, start, end);
	
	stat = cublasGetVector (NSize, sizeof(float), devPtrRes, 1, cuda_result, 1);

	if (stat != CUBLAS_STATUS_SUCCESS) {
		printf ("data upload failed");
		cudaFree (devPtrAVec);
		cublasDestroy(handle);
		return EXIT_FAILURE;
	}

	cudaFree (devPtrAVec);
	cublasDestroy(handle);

	/*float sum = 0;
	for (j = 0; j < NSize; j++) {
			sum += cuda_result[j]*AVec[j];
	}*/

	float sumCPU = 0; 
	clock_t ini = clock();
	for(int i = 0; i< NSize; i++)
	{
		float sumCPUTemp = 0;

		for(int j = 0; j< NSize; j++)
		{
			sumCPUTemp += BComplete[i*NSize+j]*AVec[j];
		}

		sumCPU+= sumCPUTemp*AVec[i];
	}
	clock_t fin = clock();

	float sumDot = 0;
	clock_t ini2 = clock();
	for(int ii = 0; ii< 1000; ii++)
	{
		for(int jj = 0; jj< NSize; jj++)
		{
			sumDot += BComplete[jj*NSize+ii]*AVec[jj];
		}
	}
	clock_t fin2 = clock();

	printf("Multiplicacion mxv de %d elems.\n", NSize );
	printf("Sumatorio completo en CUDA: %f en %f + %f ms.\n", *sum, time, time2);
	printf("Sumatorio completo en CPU: %f en %f y %f ms.\n", sumCPU, ((double)(fin-ini))/CLOCKS_PER_SEC*1000, ((double)(fin2-ini2))/CLOCKS_PER_SEC*1000);

	return EXIT_SUCCESS;
}


//
//	main()
//
int ejecutar_sgemmNN(int items)//( int argc, char **argv )
{	
/*
    int N = items;
	if(N < 16)
		N = 16;

	//FILE* cout;
	//cout = fopen("C:\\Users\\chus\\Documents\\dev\\Data\\models\\multmatrix.txt", "a");

    //
    //  startup
    //
    int idevice = 0;


    Q( cudaSetDevice( idevice ) );

    struct cudaDeviceProp prop;
    Q( cudaGetDeviceProperties( &prop, idevice ) );
    printf( "\nDevice: %s, %.0f MHz clock, %.0f MB memory.\n", prop.name, prop.clockRate/1000.f, prop.totalGlobalMem/1024.f/1024.f );

    cudaEvent_t start, end;
    Q( cudaEventCreate( &start ) );
    Q( cudaEventCreate( &end ) );

    //Q( cublasInit( ) );

    //
    //  allocate memory
    //
    //float *A = (float*)malloc( N*N*sizeof( float ) );
    float *B = (float*)malloc( N*N*sizeof( float ) );
	float *AVec = (float*)malloc( N*sizeof( float ) );

    //float *C = (float*)malloc( N*N*sizeof( float ) );
    //float *cublas_result = (float*)malloc( N*N*sizeof( float ) );
    //float *our_result = (float*)malloc( N*N*sizeof( float ) );

	float *our_result_for_sum = (float*)malloc( N*sizeof( float ) );
	float *our_result= (float*)malloc( N*sizeof( float ) );

    //fill( A, N*N, 31 );
    fill( B, N*N, 31 );
    //fill( C, N*N, 31 );
	fill( AVec, N, 31 );

	for(int i = 0; i< N; i++)
		our_result_for_sum[i] = 0;

    float *dA, *dB, *dC, *dAVec, *dCVec;
    //Q( cublasAlloc( N*N, sizeof(float), (void**)&dA ) );
    Q( cublasAlloc( N*N, sizeof(float), (void**)&dB ) );
    //Q( cublasAlloc( N*N, sizeof(float), (void**)&dC ) );
	Q( cublasAlloc( N, sizeof(float), (void**)&dAVec ) );
	Q( cublasAlloc( N, sizeof(float), (void**)&dCVec ) );
    //Q( cudaMemcpy( dA, A, N*N*sizeof(float), cudaMemcpyHostToDevice ) );
    Q( cudaMemcpy( dB, B, N*N*sizeof(float), cudaMemcpyHostToDevice ) );
	Q( cudaMemcpy( dAVec, AVec, N*sizeof(float), cudaMemcpyHostToDevice ) );
    Q( cudaMemcpy( dCVec, our_result_for_sum, N*sizeof(float), cudaMemcpyHostToDevice ) );
		
    //
    //	bench square matrices
    //
	int i = 0;
    //for( int i = 0; i < 2; i++ )
    {
        const char transa = 'N';
        const char transb = i ? 'T' : 'N';

        //printf( "\ntesting sgemm( '%c', '%c', n, n, n, ... )\n\n", transa, transb );

        const int nb = 64;
        //printf( "   n   CUBLAS,Gflop/s   we,Gflop/s   \"error\"\n" );
        int idim = 1;
		//for(idim = 1; idim <= N/nb; idim = int((idim+1)*1.25) )
        //{
			idim = N/nb;
            int dim = idim*nb;

            //
            //	set up the parameters
            //
            const int m = dim;
            const int n = dim;
            const int k = dim;
            const int lda = dim;
            const int ldb = dim;
            const int ldc = dim;
            const float alpha = 1;
            const float beta = -1;

            //
            // compute with CUBLAS
            //
			
			/*			
            Q( cublasSetMatrix( m, n, sizeof( float ), C, ldc, dC, ldc ) );
			clock_t ini1 = clock();
            cublasSgemm( transa, transb, m, n, k, alpha, dA, lda, dB, ldb, beta, dC, ldc );
			cudaError_t cudaStatus = cudaDeviceSynchronize();
			clock_t fin1 = clock();
						
            Q( cublasGetError( ) );
            Q( cublasGetMatrix( m, n, sizeof( float ), dC, ldc, cublas_result, ldc ) );
			
            //
            // compute with our routine
            //
            Q( cublasSetMatrix( m, n, sizeof( float ), C, ldc, dC, ldc ) );
			clock_t ini2 = clock();
            ourSgemm( transa, transb, m, n, k, alpha, dA, lda, dB, ldb, beta, dC, ldc );
			cudaError_t cudaStatus2 = cudaDeviceSynchronize();
			clock_t fin2 = clock();
			Q( cublasGetMatrix( m, n, sizeof( float ), dC, ldc, our_result, ldc ) );
			*/

			/*
		    dim3 grid( m/16 , 0, 0), threads( 16 , 0, 0);

			
			//Q( cublasSetMatrix( m, n, sizeof( float ), C, ldc, dC, ldc ) );
			clock_t ini3 = clock();
			vec_mat_vec_mult<<<grid, threads>>>( AVec, lda, B, ldb, dCVec, k, alpha, beta );
			clock_t fin3 = clock();

			cudaError_t cudaStatus = cudaDeviceSynchronize();
			if (cudaStatus != cudaSuccess) 
			{
				printf("cudaDeviceSynchronize returned error code %d after launching addKernel!\n", cudaStatus);
				printf("Error String: %s\n", cudaGetErrorString(cudaStatus));
			}

			Q( cudaMemcpy( our_result_for_sum, dCVec, (m/16)*sizeof(float), cudaMemcpyDeviceToHost ) );
			
			float resFinal = 0;
			for(int i = 0; i< (m/16); i++)
			{
				resFinal += our_result_for_sum[i];
			}

			printf("Resultado Final en cuda: %f en %f ms\n", resFinal, ((double)(fin3-ini3))/CLOCKS_PER_SEC*1000);
			
            //
            //	check the difference in results
            //
            //float difference = diff( m, n, cublas_result, ldc, our_result, ldc );

            //
            //	bench cublas
            //
            /*
			double cublas_time;
            cublasSgemm( transa, transb, m, n, k, alpha, dA, lda, dB, ldb, beta, dC, ldc );
            BEGIN_TIMING( );
            cublasSgemm( transa, transb, m, n, k, alpha, dA, lda, dB, ldb, beta, dC, ldc );
            END_TIMING( cublas_time );

            double cublas_gflops = 2.*m*n*k/cublas_time/1e9;

            //
            //	bench our routine
            //
            double our_time;
            ourSgemm( transa, transb, m, n, k, alpha, dA, lda, dB, ldb, beta, dC, ldc );
            BEGIN_TIMING( );
            ourSgemm( transa, transb, m, n, k, alpha, dA, lda, dB, ldb, beta, dC, ldc );
            END_TIMING( our_time );
			
            double our_gflops = 2.*m*n*k/our_time/1e9;
			*/
            //
            //	report the results
            //
			
			/*
			clock_t ini = clock();

			float result = 0;

			for(int i = 0; i< n; i++)
			{
				double tempSum = 0.0;
				for(int k = 0; k< n; k++)
				{
					tempSum += AVec[k]*B[i*n+k];
				}
				result += tempSum*AVec[i];  
			}

			clock_t fin = clock();

			printf("Resultado Final en cpu: %f en %f ms\n", result, ((double)(fin-ini))/CLOCKS_PER_SEC*1000);
			

			//printf( "TIME: %5d %11.2f %14.2f\n", n, cublas_time, our_time);
			//double time1 = ((double)(fin1-ini1))/CLOCKS_PER_SEC * 1000.0;
			//double time2 = ((double)(fin2-ini2))/CLOCKS_PER_SEC * 1000.0;
			//printf( "TIME MINE: %d, %f, %f, CPU: %f \n", n, time1, time2, ((double)(fin-ini))/CLOCKS_PER_SEC * 1000.0);
            //printf( "%5d %11.2f %14.2f %8g\n", n, cublas_gflops, our_gflops, difference );

			//fprintf(cout, "%d, %f, %f, %f \n", n, time1, time2 , ((double)(fin-ini))/CLOCKS_PER_SEC * 1000.0);
			//fflush(cout);

			
        //}
    }

	//fclose(cout);
	
    //
    //	shutdown
    //

    //cublasFree( dAVec );
    //cublasFree( dB );
    //cublasFree( dCVec );

    free( AVec );
    free( B );
    free( our_result_for_sum );

    //free( cublas_result );
    free( our_result );

    //Q( cublasShutdown( ) );

    return 0;
	*/
	return 0;
}

	

int ejecutar_matrixVector(int items)//( int argc, char **argv )
{	
    int N = items;
	if(N < 16)
		N = 16;

	//FILE* cout;
	//cout = fopen("C:\\Users\\chus\\Documents\\dev\\Data\\models\\multmatrix.txt", "a");

    int idevice = 0;
    Q( cudaSetDevice( idevice ) );

    struct cudaDeviceProp prop;
    Q( cudaGetDeviceProperties( &prop, idevice ) );
    printf( "\nDevice: %s, %.0f MHz clock, %.0f MB memory.\n", prop.name, prop.clockRate/1000.f, prop.totalGlobalMem/1024.f/1024.f );

    cudaEvent_t start, end;
    Q( cudaEventCreate( &start ) );
    Q( cudaEventCreate( &end ) );

    //
    //  allocate memory
    //

	printf("%d Elementos.\n", N);

    // Matriz simetrica, row-mayor
	float *B = (float*)malloc( (N*(N+1))/2*sizeof( float ) );
	float *BComplete = (float*)malloc( N*N*sizeof( float ) );

	// Vector
	float *AVec = (float*)malloc( N*sizeof( float ) );

	// resultados
	float *cuda_result = (float*)malloc( N*sizeof( float ) );
	float *cpu_result= (float*)malloc( N*sizeof( float ) );

	// Inicializacion
	fill( BComplete, N*N, 31 );
	fill( AVec, N, 31 );

	int count = 0;
	for(int i = 0; i< N; i++)
	{
		for(int j= i; j< N; j++)
		{
			B[count] = BComplete[i*N + j ];
			count++;
		}
	}

	for(int i = 0; i< N; i++) cuda_result[i] = 0;

    float *dA, *dB, *dAVec, *dCVec, *cuda_final_result, *cudaFinalResFloat;

	cudaError_t cudaStat; 
    //cudaStat = cudaMalloc( (void**)&dB, (N*(N+1))/2 * sizeof(float) );
	cudaStat = cudaMalloc( (void**)&dB, N*N * sizeof(float) );
	cudaStat = cudaMalloc( (void**)&dAVec, N* sizeof(float) );
	cudaStat = cudaMalloc( (void**)&dCVec, N* sizeof(float) );
	cudaStat = cudaMalloc( (void**)&cudaFinalResFloat, sizeof(float) );

    //cudaMemcpy( dB, B, (N*(N+1))/2*sizeof(float), cudaMemcpyHostToDevice );
	cudaMemcpy( dB, BComplete, N*N*sizeof(float), cudaMemcpyHostToDevice );	
	cudaMemcpy( dAVec, AVec, N*sizeof(float), cudaMemcpyHostToDevice );
    cudaMemcpy( dCVec, cuda_result, N*sizeof(float), cudaMemcpyHostToDevice );
		
    //	set up the parameters
	const int dim = N;
    const int m = dim;
    const int n = dim;
    const int k = dim;
    const int lda = dim;
    const int ldb = dim;
    const int ldc = dim;
    const float alpha = 1;
    const float beta = 0;


    // compute with CUBLAS
	clock_t ini1 = clock();
	//for(int tempI = 0; tempI< 1000; tempI++)
	{
		/*
		cublasSsymv( h, CUBLAS_FILL_MODE_UPPER, N, &alpha, dB, lda, dAVec, 1, &beta, dCVec, 1 );
		cudaError_t cudaStatus = cudaDeviceSynchronize();
		if (cudaStatus != cudaSuccess) 
		{
			printf("cudaDeviceSynchronize returned error code %d after launching addKernel!\n", cudaStatus);
			printf("Error String: %s\n", cudaGetErrorString(cudaStatus));
		}

		Q( cudaMemcpy( cuda_result, dCVec, N*sizeof(float), cudaMemcpyDeviceToHost ) );

		printf("Que pasa:\n %f %f %f %f %f \n", cuda_result[0], cuda_result[1], cuda_result[2], cuda_result[3], cuda_result[4]);

		cublasSdot( h, N, dAVec, 1, dCVec, 1 , cudaFinalResFloat);

		cudaMemcpy( &hostRes, cudaFinalResFloat, sizeof(float), cudaMemcpyDeviceToHost );
		*/

		dim3 grid(N/32), threads(32);
		vmv<<<grid, threads>>>( dAVec, lda, dB, ldb, dCVec, k, alpha, beta );

	}
	cudaError_t cudaStatus = cudaDeviceSynchronize();

	Q( cudaMemcpy( cuda_result, dCVec, N*sizeof(float), cudaMemcpyDeviceToHost ) );
	float res001 = 0;
	for(int f = 0; f < N; f++)
		res001 += cuda_result[f];

	clock_t fin1 = clock();
	printf("Resultado Final en cuda: %f en %f ms\n", res001, ((double)(fin1-ini1))/CLOCKS_PER_SEC*1000);
	float masHostRes = 0, masHostRes2 = 0;

	/*
	clock_t ini2 = clock();
	for(int tempI = 0; tempI< 1000; tempI++)
		cublasSdot( h, N, dAVec, 1, dAVec, 1, cudaFinalResFloat);
	clock_t fin2 = clock();
	printf("masHostRes:%f en %fms\n", masHostRes, ((double)(fin2-ini2))/CLOCKS_PER_SEC);

	clock_t ini3 = clock();
	for(int tempI = 0; tempI< 1000; tempI++)
	{
		masHostRes2 = 0;
		for(int i = 0; i<N; i++)
			masHostRes2 += AVec[i]*AVec[i];
	}
	clock_t fin3 = clock();

	printf("masHostResCPU:%f  en %fms\n", masHostRes2, ((double)(fin3-ini3))/CLOCKS_PER_SEC);
	*/	
			/*			
            Q( cublasSetMatrix( m, n, sizeof( float ), C, ldc, dC, ldc ) );

						
            Q( cublasGetError( ) );
            Q( cublasGetMatrix( m, n, sizeof( float ), dC, ldc, cublas_result, ldc ) );
			
            //
            // compute with our routine
            //
            Q( cublasSetMatrix( m, n, sizeof( float ), C, ldc, dC, ldc ) );
			clock_t ini2 = clock();
            ourSgemm( transa, transb, m, n, k, alpha, dA, lda, dB, ldb, beta, dC, ldc );
			cudaError_t cudaStatus2 = cudaDeviceSynchronize();
			clock_t fin2 = clock();
			Q( cublasGetMatrix( m, n, sizeof( float ), dC, ldc, our_result, ldc ) );
			*/

			/*
		    dim3 grid( m/16 , 0, 0), threads( 16 , 0, 0);

			
			//Q( cublasSetMatrix( m, n, sizeof( float ), C, ldc, dC, ldc ) );
			clock_t ini3 = clock();
			vec_mat_vec_mult<<<grid, threads>>>( AVec, lda, B, ldb, dCVec, k, alpha, beta );
			clock_t fin3 = clock();

			cudaError_t cudaStatus = cudaDeviceSynchronize();
			if (cudaStatus != cudaSuccess) 
			{
				printf("cudaDeviceSynchronize returned error code %d after launching addKernel!\n", cudaStatus);
				printf("Error String: %s\n", cudaGetErrorString(cudaStatus));
			}

			Q( cudaMemcpy( our_result_for_sum, dCVec, (m/16)*sizeof(float), cudaMemcpyDeviceToHost ) );
			*/

		//printf("Resultado Final en cuda: %f en %f ms\n", hostRes, ((double)(fin1-ini1))/CLOCKS_PER_SEC);
			
            //
            //	check the difference in results
            //
            //float difference = diff( m, n, cublas_result, ldc, our_result, ldc );

            //
            //	bench cublas
            //
            /*
			double cublas_time;
            cublasSgemm( transa, transb, m, n, k, alpha, dA, lda, dB, ldb, beta, dC, ldc );
            BEGIN_TIMING( );
            cublasSgemm( transa, transb, m, n, k, alpha, dA, lda, dB, ldb, beta, dC, ldc );
            END_TIMING( cublas_time );

            double cublas_gflops = 2.*m*n*k/cublas_time/1e9;

            //
            //	bench our routine
            //
            double our_time;
            ourSgemm( transa, transb, m, n, k, alpha, dA, lda, dB, ldb, beta, dC, ldc );
            BEGIN_TIMING( );
            ourSgemm( transa, transb, m, n, k, alpha, dA, lda, dB, ldb, beta, dC, ldc );
            END_TIMING( our_time );
			
            double our_gflops = 2.*m*n*k/our_time/1e9;
			*/
            //
            //	report the results
            //
			
	clock_t ini = clock();

	float result = 0;

	for(int i = 0; i< N; i++)
	{
		float tempSum = 0.0;
		for(int k = 0; k< N; k++)
		{
			float AvecValue = AVec[k];
			float BValue = BComplete[i*N+k];
			float tempValue = AVec[k]*BComplete[i*N+k];
			tempSum += tempValue;
		}
		result += tempSum*AVec[i];  
	}

	clock_t fin = clock();

	printf("Resultado Final en cpu: %f en %f ms\n", result, ((double)(fin-ini))/CLOCKS_PER_SEC*1000);
			

			//printf( "TIME: %5d %11.2f %14.2f\n", n, cublas_time, our_time);
			//double time1 = ((double)(fin1-ini1))/CLOCKS_PER_SEC * 1000.0;
			//double time2 = ((double)(fin2-ini2))/CLOCKS_PER_SEC * 1000.0;
			//printf( "TIME MINE: %d, %f, %f, CPU: %f \n", n, time1, time2, ((double)(fin-ini))/CLOCKS_PER_SEC * 1000.0);
            //printf( "%5d %11.2f %14.2f %8g\n", n, cublas_gflops, our_gflops, difference );

			//fprintf(cout, "%d, %f, %f, %f \n", n, time1, time2 , ((double)(fin-ini))/CLOCKS_PER_SEC * 1000.0);
			//fflush(cout);

			
        //}
    //}

	//fclose(cout);
	
    //
    //	shutdown
    //

	//cublasDestroy(h);
    cudaFree( dAVec );
    cudaFree( dB );
    cudaFree( dCVec );

    free( AVec );
    free( B );
    free( cuda_result );

    //free( cublas_result );
    free( cpu_result );


    return 0;
}	


//////////////////////
