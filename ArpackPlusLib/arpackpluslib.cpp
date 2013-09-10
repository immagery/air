#include "arpackpluslib.h"

//#include "matrices/nonsym/lnmatrxc.h"
//#include "matrices/nonsym/lnmatrxd.h"
#include "arlnsmat.h"
#include "arlgnsym.h"
#include "arlgsym.h"
#include "matrices/nonsym/lnsymsol.h"


ArpackPlusLib::ArpackPlusLib()
{


}

/*
    // Emulation of the instruction eigs from MATLAB
    // based on the ugly library arpack

      [V,D,flag] = eigs(A, T, eigno, 0, options);

      A: sparse matrix
      T: sparse matrix
      eigno: number of largest eigen values
      0: sigma
      options: verbose displaying options

      V: eigen values
      D: eigen vectors
      flag: converge condition flag

*/

/*
   2) Data structure used to represent matrices A and B:

   {nnzA, irowA, pcolA, valA}: matrix A data in CSC format.
   {nnzA, irowA, pcolA, valA}: matrix B data in CSC format.

   The SuperLU package is called by ARluNonSymGenEig to solve
   some linear systems involving B.
*/

double testMassMatrix(int n, sparseMatrix& Ain)
{
     int   i,j;
     double diag, sub;

      // Defining constants.

      sub  = 1.0/6.0/double(n+1);
      diag = 4.0/6.0/double(n+1);

      // Defining the number of nonzero matrix elements.
      Ain.nnz = 3*n-2;

      // Creating output vectors.

      Ain.val.resize(Ain.nnz, 0);
      Ain.irow.resize(Ain.nnz, 0);
      Ain.pcol.resize(n+1, 0);

      // Filling A, irow and pcol.
      Ain.pcol[0] = 0;
      j = 0;
      for (i=0; i!=n; i++) {
        if (i != 0) {
          Ain.irow[j] = i-1;
          Ain.val[j++]  = sub;
        }
        Ain.irow[j] = i;
        Ain.val[j++]  = diag;
        if (i != (n-1)) {
          Ain.irow[j] = i+1;
          Ain.val[j++]  = sub;
        }
        Ain.pcol[i+1] = j;
      }
      return 0;
}

double testStiffnessMatrix(int n, double rho, sparseMatrix& matrix)
{
    int   i, j;
    double dd, dl, du, s, h;

    // Defining constants.

    const double one = 1.0;
    const double two = 2.0;

    h  = one/double(n+1);
    s  = rho/two;
    dd = two/h;
    dl = -one/h - s;
    du = -one/h + s;

    // Defining the number of nonzero matrix elements.
    matrix.nnz = 3*n-2;

    // Creating output vectors.

    matrix.val.resize(matrix.nnz, 0);
    matrix.irow.resize(matrix.nnz, 0);
    matrix.pcol.resize(n+1, 0);

    // Filling A, irow and pcol.

    matrix.pcol[0] = 0;
    j = 0;
    for (i=0; i!=n; i++) {
      if (i != 0) {
        matrix.irow[j] = i-1;
        matrix.val[j++]  = du;
      }
      matrix.irow[j] = i;
      matrix.val[j++]  = dd;
      if (i != (n-1)) {
        matrix.irow[j] = i+1;
        matrix.val[j++]  = dl;
      }
      matrix.pcol[i+1] = j;
    }

    return 0;
}

int eigs(int dimension, sparseMatrix& Ain, sparseMatrix& Bin, int eigno, double sigma,
            vector<double>& eigenValues, vector< vector<double> >& eigenVectors, double tol)
{

    // Creating matrices A and B.
    ARluSymMatrix<double> A(dimension, Ain.nnz, &Ain.val[0], &Ain.irow[0], &Ain.pcol[0]);
    ARluSymMatrix<double> B(dimension, Bin.nnz, &Bin.val[0], &Bin.irow[0], &Bin.pcol[0]);

    ARluSymGenEig<double> dprob(eigno, A, B, "SM", eigno*2, tol);

    // Finding eigenvalues and eigenvectors.
    dprob.FindEigenvectors();

    int n = dprob.GetN();
    int nconv = dprob.ConvergedEigenvalues();

    // Collect the solution
    eigenValues.resize(nconv);

    for (int i=0; i<nconv; i++)
    {
        eigenValues[i] = dprob.Eigenvalue(nconv-i-1);
    }

    eigenVectors.resize(n);
    for (int i=0; i<n; i++)
    {
        eigenVectors[i].resize(nconv,0);
        for (int j=0; j<nconv; j++)
        {
            eigenVectors[i][nconv-j-1] = dprob.Eigenvector(j,i);
        }
    }

    if(OWN_VERBOSE)
    {
        std::cout << "Dimension of the system  : " << n             << std::endl;
        std::cout << "'requested' eigenvalues  : " << dprob.GetNev() << std::endl;
        std::cout << "'converged' eigenvalues  : " << nconv         << std::endl;
        std::cout << "Arnoldi vectors generated: " << dprob.GetNcv() << std::endl;
        std::cout << std::endl;
    }

    return (eigno-nconv);
}

