#ifndef INTERIORDISTANCES_H
#define INTERIORDISTANCES_H

#include <DataStructures\DataStructures.h>

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#define EIGEN_SUPERLU_SUPPORT
//#define EIGEN_GOOGLEHASH_SUPPORT

#define VERBOSE_ID true

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseExtra>
#include <Eigen/SuperLUSupport>

using Eigen::MatrixXd;
using Eigen::Matrix3f;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::Vector3f;
using Eigen::Vector3d;

using namespace std;
using namespace Eigen;


/*   ---- EIGEN VECTOR SOLUTION PROCESS ---
  Computes data for process embedding of any interior points from model
  vertices eigen vectors interpolation

  >  modelo                 -- model for compute embedding.
  >  eigenValuesRes         -- result (eigen values from each model vertex)
  >  eigenVectorsRes        -- result (eigen vector from each model vertex)

*/
int getEigenVectorSolution(MyMesh& modelo,
                           vector<double>& eigenValuesRes,
                           vector< vector<double> >& eigenVectorsRes);

int interiorDistancesTest(string fileName, MyMesh& modelo, vector<vcg::Point3d>& points, vector<double>& distances, bool bVerbose);

int interiorDistancesFromSimplificationTest(string prefix, string fileNameMedResEmbedding,string fileNameHiResEmbedding,
                                            MyMesh& modeloMedRes, MyMesh& modeloHiRes,vector<vcg::Point3d>& points,
                                            vector<double>& distances, bool bVerbose = true);

#endif // INTERIORDISTANCES_H
