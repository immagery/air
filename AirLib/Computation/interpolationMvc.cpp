//Raif Rustamov, Drew University
//evaluate 3D mean value interpolant
//inputs must be transposed!!!! remember fortran C differenc row first vs columns first
//follows the pseudocode in Ju et al.
#include <math.h>
#define  PI  (3.14159265)

//these are declared global to save time for mallocing
double *output;
double *totalF;
//distance from vertices to the x
double *d;
//unit vector from x to a mesh vertex
double (*u)[3];


double distance(double pt1[3], double pt2[3]){
        double res=0;
        int i;
        for (i=0; i<3; i++){
                res += (pt2[i]-pt1[i])*(pt2[i]-pt1[i]);
        }
        return sqrt(res);
}

double length(double pt[3]){
        double res=0;
        int i;
        for (i=0; i<3; i++){
                res += pt[i]*pt[i];
        }
        return sqrt(res);
}

double sign_det(double v0[3], double v1[3], double v2[3])
//return sign(det[v1 v2 v3])
{
        double det = v0[0]*v1[1]*v2[2] + v0[1]*v1[2]*v2[0] + v0[2]*v1[0]*v2[1] - v0[2]*v1[1]*v2[0] - v0[0]*v1[2]*v2[1]-v0[1]*v1[0]*v2[2];
        if(det >= 0.0) return 1.0;
        else return -1.0;
}

double *mvc(double x[3], double mesh_funcs[], int nofuncs, double mesh_coord[][3], double mesh_triang[][3], int nopts, int notrg)
//this does the job for one point p, returns the value of the interpolant at this point
//the output is an array of size 1 x nofuncs
//mesh_funcs[i*nopts + j] contains the value of i-th function at vertex j
{
        const double tresh1 = 0.0001;
        const double tresh2 = 0.001;
        const double tresh3 = 0.0001;

        int i, t, f, k;
        double totalW;

        double dist;
        double l[3], theta[3], w[3], s[3], c[3];
        int vert[3];
        double h, sss;

        //mexPrintf("vert=%g %g %g\n", x[0], x[1], x[2]);

        for(i=0; i<nopts; i++){
                dist = distance(x, mesh_coord[i]);
                if (dist<tresh1){
                        //very close to a vertex, simply return the value at the vertex
                        for (f=0; f<nofuncs; f++){
                                output[f] = mesh_funcs[f+ nofuncs*i];
                        }
                        return output;
                }
                u[i][0] = (mesh_coord[i][0] - x[0])/dist;
                u[i][1] = (mesh_coord[i][1] - x[1])/dist;
                u[i][2] = (mesh_coord[i][2] - x[2])/dist;
                d[i] = dist;
        }


        for (f=0; f<nofuncs; f++){
                totalF[f] = 0;
        }
        totalW = 0;


        for(t=0; t< notrg; t++){

                //mexPrintf("t=%d\n", t);

                for (k=0; k<3; k++)
                        vert[k] = (int) mesh_triang[t][k] - 1; //triangle vertex indices

                //mexPrintf("vert=%d %d %d\n", vert[0], vert[1], vert[2]);

                l[0] = distance(u[vert[1]], u[vert[2]]);
                l[1] = distance(u[vert[2]], u[vert[0]]);
                l[2] = distance(u[vert[0]], u[vert[1]]);

                //mexPrintf("l are %g %g %g \n", l[0], l[1], l[2]);

                for (k=0; k<3; k++){
                        theta[k] = 2*asin(l[k]/2);
                }

                h = (theta[0] + theta[1] + theta[2])/2;
                if(PI - h < tresh2){
                        //x lies within the triangle, use 2D MVcoords
                        w[0] = sin(theta[0])*d[vert[1]]*d[vert[2]];
                        w[1] = sin(theta[1])*d[vert[0]]*d[vert[2]];
                        w[2] = sin(theta[2])*d[vert[1]]*d[vert[0]];
                        for (f = 0; f<nofuncs; f++){
                                output[f] = (w[0]*mesh_funcs[f+ nofuncs*vert[0]] + w[1]*mesh_funcs[f+ nofuncs*vert[1]]+w[2]*mesh_funcs[f+ nofuncs*vert[2]])/(w[0]+w[1]+w[2]);
                        }
                        return output;
                }
                c[0] =2*sin(h)*sin(h-theta[0])/(sin(theta[1])*sin(theta[2]))-1;
                c[1] =2*sin(h)*sin(h-theta[1])/(sin(theta[2])*sin(theta[0]))-1;
                c[2] =2*sin(h)*sin(h-theta[2])/(sin(theta[0])*sin(theta[1]))-1;


                sss = sign_det(u[vert[0]], u[vert[1]], u[vert[2]]);

                for(k=0; k<3; k++){
                        s[k] = sss*sqrt(1-c[k]*c[k]);
                }
                //mexPrintf("s are %g %g %g \n", s[0], s[1], s[2]);
                if((fabs(s[0]) > tresh3) && (fabs(s[1]) > tresh3) && (fabs(s[2]) > tresh3) ){
                        //if any is less thatn tresh then no contribution
                        //belongs to the same plane but outside
                        w[0] = (theta[0] - c[1]*theta[2] - c[2]*theta[1])/(d[vert[0]]*sin(theta[1])*s[2]);
                        w[1] = (theta[1] - c[0]*theta[2] - c[2]*theta[0])/(d[vert[1]]*sin(theta[2])*s[0]);
                        w[2] = (theta[2] - c[1]*theta[0] - c[0]*theta[1])/(d[vert[2]]*sin(theta[0])*s[1]);

                        totalW += w[0]+w[1]+w[2];
                        //mexPrintf("totalW is %g  \n", totalW);

                        for(f=0; f<nofuncs; f++){
                                totalF[f] += w[0]*mesh_funcs[f+ nofuncs*vert[0]] + w[1]*mesh_funcs[f+ nofuncs*vert[1]]+w[2]*mesh_funcs[f+ nofuncs*vert[2]];
                        }
                }
        }

        //mexPrintf("totalW is %g  \n", totalW);
        for(f=0; f<nofuncs; f++)
                output[f] = totalF[f]/totalW;

        return output;
}



/* The gateway routine */
void mexFunction(int nlhs, mxArray *plhs[],
                                 int nrhs, const mxArray *prhs[])
{
        double (*interior_points)[3];
        double * mesh_funcs;
        double (*mesh_coord)[3];
        double (*mesh_triang)[3];
        double *result;
        double *mvc_out;

        int i, j, no_interior_points, nofuncs, nopts, notrg;

        if (nrhs != 4)
                mexErrMsgTxt("Four inputs required.");
        if (nlhs != 1)
                mexErrMsgTxt("One output required.");

        //row <---> col
        interior_points = (double(*)[3]) mxGetPr(prhs[0]);
        no_interior_points = mxGetN(prhs[0]); //number of rows is the number of int. points, second dim =3

        mesh_funcs = mxGetPr(prhs[1]);
        nofuncs = mxGetM(prhs[1]); //number of columns is the number of functions, #rows = no_interior_points

        mesh_coord = (double(*)[3]) mxGetPr(prhs[2]);
        nopts = mxGetN(prhs[2]); //rows

        mesh_triang = (double (*)[3]) mxGetPr(prhs[3]);
        notrg = mxGetN(prhs[3]); //rows

        //mexPrintf("nos are %d %d %d  %d\n", no_interior_points, nofuncs, nopts, notrg);

        /* Set the output pointer to the output matrix. */
        plhs[0] = mxCreateDoubleMatrix(nofuncs, no_interior_points,  mxREAL);

        /* Create a C pointer to a copy of the output matrix. */
        result = mxGetPr(plhs[0]);
        if(result == NULL)
                mexPrintf("not enough memory\n");

        //allocate the variables used in the function
        output = (double *) malloc(nofuncs*sizeof(double));
        totalF = (double *) malloc(nofuncs*sizeof(double));
        //distance from vertices to the x
        d = (double *) malloc(nopts*sizeof(double));
        //unit vector from x to a mesh vertex
        u = (double(*)[3]) malloc(nopts*sizeof(*u));


        //now for each interior point will evaluate the MVC interpolant, and put into result
        for(i=0; i<no_interior_points; i++){
                //mexPrintf("i=%d\n", i);
                mvc_out = mvc(interior_points[i], mesh_funcs, nofuncs, mesh_coord, mesh_triang, nopts, notrg);
                for (j=0; j<nofuncs; j++){
                        result[j + nofuncs*i] = mvc_out[j];
                }
        }
        //free all of the variables made
        free(totalF);
        free(d);
        free(u);
        free(output);
}


