#ifndef GREENCOORDS_H
#define GREENCOORDS_H

#include <vector>
#include <DataStructures\DataStructures.h>

using namespace std;

// Obtiene el signo de un numero
//double sign(double v);

// Calculo de una parte de las coordenadas
//double GCTriInt( Vector3d p,  Vector3d v1,  Vector3d v2,  Vector3d pt);

// Calculo de las coordenadas de green para un modelo y una caja envolvente.
//void gpCalculateGreenCoordinates(MyMesh& mesh, MyMesh& cage, std::vector< std::vector<float> > &PerVertGC, std::vector< std::vector<float> > &PerFaceGC, string sSavingFile);

//void deformMeshWithGC(MyMesh& mesh, MyMesh& cage, MyMesh& newMesh, MyMesh& newCage, std::vector< std::vector<float> >& PerVertGC, std::vector< std::vector<float> >& PerFaceGC);

void LoadGCCoordsFromFile(std::vector< std::vector<float> > &PerVertGC, std::vector< std::vector<float> > &PerFaceGC, string sFileName);
void SaveGCCoordsToFile(std::vector< std::vector<float> > &PerVertGC, std::vector< std::vector<float> > &PerFaceGC, string sFileName);

#endif // GREENCOORDS_H
