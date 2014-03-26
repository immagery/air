#ifndef CUDA_DEFINITIONS_H
#define CUDA_DEFINITIONS_H

#include <assert.h>

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <device_functions.h>
#include <math_constants.h>

#include <time.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <helper_cuda.h>

const int threadsPerBlock = 256;

#define PRECISION float
#define PRECISION3 float3

#endif