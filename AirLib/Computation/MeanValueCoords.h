#ifndef MEANVALUECOORDS_H
#define MEANVALUECOORDS_H

#include <vector>
#include "..\DataStructures\DataStructures.h"

using namespace std;

// Obtiene el signo de un numero
//double sign(double v);

// Calculo de una parte de las coordenadas
//double GCTriInt(vcg::Point3d p, vcg::Point3d v1, vcg::Point3d v2, vcg::Point3d pt);

// Calculo de las coordenadas de green para un modelo y una caja envolvente.
void  CalculateMeanValueCoords(MyMesh& mesh, MyMesh& cage, std::vector< std::vector<float> > &PerVertMVC, string sFileName);

void  deformMeshWithMVC(MyMesh& mesh, MyMesh& cage, MyMesh& newMesh, MyMesh& newCage, std::vector< std::vector<float> >& PerVertMVC);

void LoadMVCoordsFromFile(std::vector< std::vector<float> > &PerVertMVC, string sFileName);
void SaveMVCoordsToFile(std::vector< std::vector<float> > &PerVertMVC, string sFileName);

#endif // GREENCOORDS_H
