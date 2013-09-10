#ifndef UTIL_OPS_H
#define UTIL_OPS_H

// VCG lib
#include <vcg/complex/complex.h>

// Std lib
#include <cmath>
#include <string>
#include <vector>
#include <time.h>
#include <stdio.h>

using namespace std;

string   getCurrentTime();

void GetColour(double v,double vmin,double vmax, float& r, float& g, float& b);
void GetColourGlobal(double v,double vmin,double vmax, float& r, float& g, float& b);

float Orientation(float ox, float oy, float oz, float nx, float ny, float nz );

double timelapse(clock_t clock1,clock_t clock2);

int indexOf(vector<int>& labels, int label);

double round(double x);

double sign(double v);

double det(vcg::Point3d u1, vcg::Point3d u2, vcg::Point3d u3);

#endif // UTIL_H
