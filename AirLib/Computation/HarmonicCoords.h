#ifndef HARMONICCOORDS_H
#define HARMONICCOORDS_H

#include "..\DataStructures\grid3D.h"
#include "..\DataStructures\DataStructures.h"

#include <vector>

using namespace std;

void loadHarmonicCoordinates(MyMesh& mesh, grid3d& grid, std::vector< std::vector<float> > &PerVertHC, string sSavingFile);
void getHarmonicCoordinates(MyMesh& mesh, MyMesh& cage, grid3d& grid, std::vector< std::vector<float> > &PerVertGC, unsigned int resolution, string sSavingFile);
void deformMeshWithHC(MyMesh& mesh, MyMesh& cage, MyMesh& newMesh, MyMesh& newCage, std::vector< std::vector<float> >& PerVertHC);

#endif // HARMONICCOORDS_H
