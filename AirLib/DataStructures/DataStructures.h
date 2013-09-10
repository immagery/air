#ifndef DATASTRUCTURES_H
#define DATASTRUCTURES_H

#include <vcg/complex/complex.h>
#include<vcg/complex/append.h>

class MyVertex;
class MyFace;

class MyUsedTypes: public vcg::UsedTypes< vcg::Use<MyVertex>::AsVertexType,vcg::Use<MyFace>::AsFaceType> {};

class MyVertex : public vcg::Vertex<MyUsedTypes, vcg::vertex::Coord3d, vcg::vertex::Normal3d, vcg::vertex::VFAdj, vcg::vertex::Mark> {};
class MyFace   : public vcg::Face<MyUsedTypes, vcg::face::VertexRef, vcg::vertex::Normal3d, vcg::face::VFAdj, vcg::face::Mark> {};
class MyMesh   : public vcg::tri::TriMesh< std::vector<MyVertex>, std::vector<MyFace> > {};

void initModelForCoordinates(MyMesh& mesh);
void gpUpdateNormals(MyMesh& mesh, bool vertexAlso = false);
void buildCube(MyMesh& mesh, vcg::Point3d center, double radius);

#endif // DATASTRUCTURES_H
