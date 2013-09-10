#include "util.h"

// devuelve el signo del valor de entrada.
double sign(double v)
{
    if(v >= 0)
        return 1;
    else
        return -1;
}

double det(vcg::Point3d u1, vcg::Point3d u2, vcg::Point3d u3)
{
    return u1[0]*u2[1]*u3[2] + u2[0]*u1[2]*u3[1] + u3[0]*u1[1]*u2[2]
            - u1[2]*u2[1]*u3[0] - u2[0]*u1[1]*u3[2] - u3[1]*u1[0]*u2[2] ;
}

double round(double x)
{
	return floor(x + 0.5);
}

double timelapse(clock_t clock1,clock_t clock2)
{
    double diffticks=clock1-clock2;
    double diffms=diffticks/CLOCKS_PER_SEC;
    return diffms;
}

float Orientation(float ox, float oy, float oz,
                  float nx, float ny, float nz )
{
  float sigma = std::max(0.0f, - (ox * nx + oy * ny + oz * nz));
                          // / (depth * depth * depth));
  return sigma;
}

void GetColourGlobal(double v,double vmin,double vmax, float& r, float& g, float& b)
{
   r = g = b = 1.0; // white
   double dv;

   if(v>vmax) v = vmax;
   if(v<vmin) v = vmin;

   if(vmin >= 0)
   {
	   dv = vmax - vmin;

	   if (v < (vmin + 0.25 * dv)) {
		  r = 0;
		  g = (4 * (v - vmin) / dv);
	   } else if (v < (vmin + 0.5 * dv)) {
		  r = 0;
		  b = (1 + 4 * (vmin + 0.25 * dv - v) / dv);
	   } else if (v < (vmin + 0.75 * dv)) {
		  r = ( 4 * (v - vmin - 0.5 * dv) / dv );
		  b = (0);
	   } else {
		  g = (1 + 4 * (vmin + 0.75 * dv - v) / dv);
		  b = (0);
	   }
   }
   else
   {
	   if(v>=0)
	   {
		   dv = vmax;
		   vmin = 0;

		   if (v < (vmin + 0.25 * dv)) {
			  r = 0;
			  g = (4 * (v - vmin) / dv);
		   } else if (v < (vmin + 0.5 * dv)) {
			  r = 0;
			  b = (1 + 4 * (vmin + 0.25 * dv - v) / dv);
		   } else if (v < (vmin + 0.75 * dv)) {
			  r = ( 4 * (v - vmin - 0.5 * dv) / dv );
			  b = (0);
		   } else {
			  g = (1 + 4 * (vmin + 0.75 * dv - v) / dv);
			  b = (0);
		   }
	   }
	   else
	   {
		   v = -v;
		   vmax = -vmin;
		   dv = vmax;
		   vmin = 0;

		   b = 1-(v/dv);
		   r = g = 0;
		   /*
		   if (v < (vmin + 0.25 * dv)) {
			  r = 0;
			  g = (4 * (v - vmin) / dv);
		   } else if (v < (vmin + 0.5 * dv)) {
			  r = 0;
			  b = (1 + 4 * (vmin + 0.25 * dv - v) / dv);
		   } else if (v < (vmin + 0.75 * dv)) {
			  r = ( 4 * (v - vmin - 0.5 * dv) / dv );
			  b = (0);
		   } else {
			  g = (1 + 4 * (vmin + 0.75 * dv - v) / dv);
			  b = (0);
		   }
		   */
	   }
   }
}

void GetColour(double v,double vmin,double vmax, float& r, float& g, float& b)
{
   r = g = b = 1.0; // white
   double dv;

   if(vmin < 0 || vmax < 0 || v < 0 || v > vmax || v < vmin)
   {
       //printf("\nValores negativos v:%f, vmin:%f, vmax:%f\n", v, vmin, vmax);
       r = g = b = 0;
   }

   if (v < vmin)
      v = vmin;
   if (v > vmax)
      v = vmax;
   dv = vmax - vmin;

   if (v < (vmin + 0.25 * dv)) {
      r = 0;
      g = (4 * (v - vmin) / dv);
   } else if (v < (vmin + 0.5 * dv)) {
      r = 0;
      b = (1 + 4 * (vmin + 0.25 * dv - v) / dv);
   } else if (v < (vmin + 0.75 * dv)) {
      r = ( 4 * (v - vmin - 0.5 * dv) / dv );
      b = (0);
   } else {
      g = (1 + 4 * (vmin + 0.75 * dv - v) / dv);
      b = (0);
   }
}

int indexOf(vector<int>& labels, int label)
{
    int res = -1;
    for(unsigned int i = 0; i< labels.size(); i++)
    {
        if(labels[i] == label)
            return i;
    }

    return res;
}
