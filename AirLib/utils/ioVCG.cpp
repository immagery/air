#include "ioVCG.h"

bool readModel(MyMesh& modelo, QString fileName, bool bVerbose = false)
{
    if(bVerbose)
        printf("Lectura del modelo %s\n", fileName.toStdString().c_str()); fflush(0);

    if(!QFile(fileName).exists())
    {
        printf("No se ha podido leer el modelo\n");
        return false;
    }

    vcg::tri::io::ImporterOFF<MyMesh>::Open(modelo,fileName.toStdString().c_str());
    vcg::tri::UpdateTopology<MyMesh>::VertexFace(modelo);
    gpUpdateNormals(modelo, true);

    // inicializaci—n de id's de vertices y tri‡ngulos.
    MyMesh::VertexIterator vertIt;
    int idVert = 0;
    for(vertIt = modelo.vert.begin(); vertIt!=modelo.vert.end(); ++vertIt )
    {
        vertIt->IMark() = idVert;
        idVert++;
    }

    if(bVerbose)
        printf("%d Vertices y %d Caras\n", modelo.vn, modelo.fn); fflush(0);

    return true;
}
