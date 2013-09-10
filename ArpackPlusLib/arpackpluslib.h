#ifndef ARPACKPLUSLIB_H
#define ARPACKPLUSLIB_H

#include <vector>

#define OWN_VERBOSE false

using namespace std;

class sparseMatrix
{
public:

    vector<double>   val;
    vector<int>     pcol;  // pointers to arrays of pointers to the
                    // beginning of each column of A and B in
                    // valA and ValB.

    vector<int>     irow;  // pointers to arrays that store the row
                    // indices of the nonzeros in A and B.

    int              nnz;    // Number of nonzero elements in A and B.


    sparseMatrix(){ nnz = 0; irow.clear(); pcol.clear(); val.clear(); }

    sparseMatrix(sparseMatrix& a)
    {
        nnz = a.nnz;
        irow.resize(a.irow.size()); for(unsigned int i = 0; i< a.irow.size(); i++) irow[i] = a.irow[i];
        pcol.resize(a.pcol.size()); for(unsigned int i = 0; i< a.pcol.size(); i++) pcol[i] = a.pcol[i];
        val.resize(a.val.size());   for(unsigned int i = 0; i< a.val.size(); i++)  val[i]  = a.val[i];
    }
};


int eigs(int dimension, sparseMatrix& Ain, sparseMatrix& Bin, int eigno, double sigma,
            vector<double>& eigenValues, vector< vector<double> >& eigenVectors, double tol = 0);

class ArpackPlusLib {
public:
    ArpackPlusLib();    
};

#endif // ARPACKPLUSLIB_H
