#ifndef CUDA_LOCK_H
#define CUDA_LOCK_H

#include <cuda_runtime.h>

#include <sm_35_intrinsics.h>
#include <sm_32_atomic_functions.h>
#include <sm_11_atomic_functions.h>
#include <sm_12_atomic_functions.h>
#include <atomic>

#include <device_launch_parameters.h>
#include <device_functions.h>
#include <math_constants.h>

struct Lock
{
	int* mutex;
	Lock( void )
	{
		int state = 0;
		checkCudaErrors( cudaMalloc( (void**)&mutex, sizeof(int) ) );
		checkCudaErrors( cudaMemcpy( mutex, &state, sizeof(int), cudaMemcpyHostToDevice) );
	}

	~Lock( void )
	{
		cudaFree( mutex );
	}

	__device__ void lock( void )
	{
		#if __CUDA_ARCH__ >= 200
			while( atomicCAS(mutex, 0, 1) != 0 );
		#endif
	}

	__device__ void unlock( void )
	{
		#if __CUDA_ARCH__ >= 200 
			atomicExch( mutex, 0);
		#endif
	}

};

#endif