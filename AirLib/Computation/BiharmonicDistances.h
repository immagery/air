#ifndef BIHARMONICDISTANCES_H
#define BIHARMONICDISTANCES_H

#include "DataStructures/DataStructures.h"
#include "DataStructures/SurfaceData.h"

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#define EIGEN_SUPERLU_SUPPORT
//#define EIGEN_GOOGLEHASH_SUPPORT

//#define VERBOSE false

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseExtra>
#include <Eigen/SuperLUSupport>

int biharmonicDistances(MyMesh& modelo, std::vector<int>& indices, Eigen::MatrixXd& dists);
int bindingBD(Modelo& modelo, binding* bd, std::vector<int>& indices, symMatrix& dists, bool withPatches = false);

#endif // BIHARMONICDISTANCES_H
