#include "DataStructures.h"

void initModelForCoordinates(MyMesh& mesh)
{
    // Anadimos la posibilidad de almacenar las coordenadas en cada vertice de la malla
    // Una coordenada por cada vertice de la cage y por cada cara.
    vcg::tri::Allocator<MyMesh>::AddPerVertexAttribute<unsigned int> (mesh,std::string("vertIdx"));
    vcg::tri::Allocator<MyMesh>::AddPerFaceAttribute<unsigned int> (mesh,std::string("faceIdx"));
}

void  buildCube(MyMesh& mesh, vcg::Point3d center, double radius) {

    radius = radius /2.0;
    // generate vertices
    mesh.Clear();
    vcg::tri::Allocator<MyMesh>::AddVertices(mesh,12);
    vcg::tri::Allocator<MyMesh>::AddFaces(mesh,20);

    MyMesh::VertexPointer ivp[12];
    MyMesh::VertexIterator vi=mesh.vert.begin();
    unsigned int idx = 0;

    ivp[0]=&*vi;(*vi).P()=MyMesh::CoordType ( -radius+center.X(), -radius+center.Y(),  radius+center.Z()); ++vi;
    ivp[1]=&*vi;(*vi).P()=MyMesh::CoordType (  radius+center.X(), -radius+center.Y(),  radius+center.Z()); ++vi;
    ivp[2]=&*vi;(*vi).P()=MyMesh::CoordType (  radius+center.X(),  radius+center.Y(),  radius+center.Z()); ++vi;
    ivp[3]=&*vi;(*vi).P()=MyMesh::CoordType ( -radius+center.X(),  radius+center.Y(),  radius+center.Z()); ++vi;
    ivp[4]=&*vi;(*vi).P()=MyMesh::CoordType ( -radius+center.X(), -radius+center.Y(), -radius+center.Z()); ++vi;
    ivp[5]=&*vi;(*vi).P()=MyMesh::CoordType (  radius+center.X(), -radius+center.Y(), -radius+center.Z()); ++vi;
    ivp[6]=&*vi;(*vi).P()=MyMesh::CoordType (  radius+center.X(),  radius+center.Y(), -radius+center.Z()); ++vi;
    ivp[7]=&*vi;(*vi).P()=MyMesh::CoordType ( -radius+center.X(),  radius+center.Y(), -radius+center.Z()); ++vi;

    ivp[8]=&*vi;(*vi).P()=MyMesh::CoordType ( -radius+center.X(),  -radius+center.Y(), center.Z()); ++vi;
    ivp[9]=&*vi;(*vi).P()=MyMesh::CoordType ( -radius+center.X(),  radius+center.Y(), center.Z()); ++vi;
    ivp[10]=&*vi;(*vi).P()=MyMesh::CoordType ( radius+center.X(),  radius+center.Y(), center.Z()); ++vi;
    ivp[11]=&*vi;(*vi).P()=MyMesh::CoordType ( radius+center.X(),  -radius+center.Y(), center.Z()); ++vi;

    MyMesh::FaceIterator fi=mesh.face.begin();
    (*fi).V(0)=ivp[0];  (*fi).V(1)=ivp[1]; (*fi).V(2)=ivp[2]; ++fi;
    (*fi).V(0)=ivp[2];  (*fi).V(1)=ivp[3]; (*fi).V(2)=ivp[0]; ++fi;

    (*fi).V(0)=ivp[7];  (*fi).V(1)=ivp[6]; (*fi).V(2)=ivp[5]; ++fi;
    (*fi).V(0)=ivp[5];  (*fi).V(1)=ivp[4]; (*fi).V(2)=ivp[7]; ++fi;

    (*fi).V(0)=ivp[8];  (*fi).V(1)=ivp[0]; (*fi).V(2)=ivp[9]; ++fi;
    (*fi).V(0)=ivp[0];  (*fi).V(1)=ivp[3]; (*fi).V(2)=ivp[9]; ++fi;

    (*fi).V(0)=ivp[4];  (*fi).V(1)=ivp[8]; (*fi).V(2)=ivp[7];  ++fi;
    (*fi).V(0)=ivp[8];  (*fi).V(1)=ivp[9]; (*fi).V(2)=ivp[7];  ++fi;

    (*fi).V(0)=ivp[9];  (*fi).V(1)=ivp[3]; (*fi).V(2)=ivp[2];  ++fi;
    (*fi).V(0)=ivp[2];  (*fi).V(1)=ivp[10]; (*fi).V(2)=ivp[9]; ++fi;

    (*fi).V(0)=ivp[7];  (*fi).V(1)=ivp[9]; (*fi).V(2)=ivp[10]; ++fi;
    (*fi).V(0)=ivp[10];  (*fi).V(1)=ivp[6]; (*fi).V(2)=ivp[7]; ++fi;

    (*fi).V(0)=ivp[1];  (*fi).V(1)=ivp[10]; (*fi).V(2)=ivp[2];  ++fi;
    (*fi).V(0)=ivp[1];  (*fi).V(1)=ivp[11]; (*fi).V(2)=ivp[10]; ++fi;

    (*fi).V(0)=ivp[11];  (*fi).V(1)=ivp[5]; (*fi).V(2)=ivp[10]; ++fi;
    (*fi).V(0)=ivp[5];  (*fi).V(1)=ivp[6]; (*fi).V(2)=ivp[10];  ++fi;

    (*fi).V(0)=ivp[8];  (*fi).V(1)=ivp[4]; (*fi).V(2)=ivp[5];  ++fi;
    (*fi).V(0)=ivp[8];  (*fi).V(1)=ivp[5]; (*fi).V(2)=ivp[11]; ++fi;

    (*fi).V(0)=ivp[0];  (*fi).V(1)=ivp[8]; (*fi).V(2)=ivp[11]; ++fi;
    (*fi).V(0)=ivp[0];  (*fi).V(1)=ivp[11]; (*fi).V(2)=ivp[1]; ++fi;


    gpUpdateNormals(mesh, false);
    return;
}

// Calcula las normales por cara y vertice opcionalmente.
// Asigna tambien indices a todos los vertices y caras
void gpUpdateNormals(MyMesh& mesh, bool vertexAlso) {

    // Calculamos la normal y ponemos
    // un indice a cada cara.
    MyMesh::FaceIterator fi;
    int idx = 0;
    for(fi = mesh.face.begin(); fi!=mesh.face.end(); ++fi ) {
         vcg::Point3d pt0 = (*fi).P(0);
         vcg::Point3d pt1 = (*fi).P(1);
         vcg::Point3d pt2 = (*fi).P(2);

         vcg::Point3d n = (pt1-pt0)^(pt2-pt0);
         fi->N() = n.normalized();
         fi->IMark() = idx;
         idx++;
     }

    // Ponemos un indice a cada vertice.
    idx= 0;
    MyMesh::VertexIterator vii;
    for(vii = mesh.vert.begin(); vii!=mesh.vert.end(); ++vii ) {
        vii->IMark() = idx;
        idx++;
     }

    // calculamos tambien las normales por vertice.
    if(vertexAlso) {
        MyMesh::VertexIterator vi;
        for(vi = mesh.vert.begin(); vi!=mesh.vert.end(); ++vi ) {
             vcg::Point3d n(0,0,0);
             int vCounter = 0;

             vcg::face::VFIterator<MyFace> vfi(&(*vi)); //initialize the iterator to the first face
             for(;!vfi.End();++vfi) {
               MyFace* f = vfi.F();
               n += f->N();
               vCounter++;
             }
             vi->N() = n/vCounter;
         }
    }
}

