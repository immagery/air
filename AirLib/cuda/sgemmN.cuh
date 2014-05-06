#ifndef CUDA_SGEMMN_H
#define CUDA_SGEMMN_H

// Written by Vasily Volkov.
// Copyright (c) 2008, The Regents of the University of California. 
// All rights reserved.

#include <stdio.h>
#include "cuda.h"

#define TIMER_TOLERANCE 0.1f

#define BEGIN_TIMING( )	\
{\
    unsigned int n_iterations;	\
    for( n_iterations = 1; n_iterations < 0x80000000; n_iterations *= 2 )\
    {\
        Q( cudaThreadSynchronize( ) );\
        Q( cudaEventRecord( start, 0 ) );\
        for( unsigned int iteration = 0; iteration < n_iterations; iteration++ ){

#define END_TIMING( seconds ) }\
        Q( cudaEventRecord( end, 0 ) );\
        Q( cudaEventSynchronize( end ) );\
        float milliseconds;\
        Q( cudaEventElapsedTime( &milliseconds, start, end ) );\
        seconds = milliseconds/1e3f;\
        if( seconds >= TIMER_TOLERANCE )\
            break;\
    }\
    seconds /= n_iterations;\
}

#define Q( condition ) {if( (condition) != 0 ) { printf( "\n FAILURE in %s, line %d\n", __FILE__, __LINE__ );exit( 1 );}}

int ejecutar_sgemmNN( int items/*int argc, char **argv */);	
int ejecutar_matrixVector( int items/*int argc, char **argv */);	

int cublas_example();

#endif