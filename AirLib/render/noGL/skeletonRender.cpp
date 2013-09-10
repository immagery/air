#include "..\SkeletonRender.h"

#define NORMALR 0.5
#define NORMALG 0
#define NORMALB 0.5

#define SELR 0.1
#define SELG 0.8
#define SELB 0.1

#define SUBSELR 1.0
#define SUBSELG 1.0
#define SUBSELB 1.0

// render size
#define DEFAULT_SIZE 0.05

#include "..\..\DataStructures/Skeleton.h"

void SkeletonRender::drawFunc(skeleton* obj)
{
}

bool SkeletonRender::updateSkeleton(skeleton* skt)
{
	assert(false);
    return true;
}


void drawLine(double x, double y, double z)
{
}

void drawBone(double l, double r)
{
}

//JOINT
void JointRender::drawFunc(joint* jt)
{    
}

void JointRender::computeWorldPos(joint* jt)
{
}