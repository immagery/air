#include "ioWeights.h"

void saveWeightsToMaya(Geometry* mesh, vector< vector<float> >& meshWeights,vector<string>& names, vector<Point3d>& points, string sFile)
{
	//TODO
	assert(false);
	/*
    FILE* fout;
    fout = fopen(sFile.c_str(), "w");

    // Cuantos nodos actuan?
    fprintf(fout, "Number of Transforms:\n");
    fprintf(fout, "%d\n\n", points.size());

    // Cuantos vertices tiene la malla?
    fprintf(fout, "Number of points:\n");
    fprintf(fout, "%d\n\n", mesh->vn);

    // Los nombres de los nodos
    fprintf(fout, "Transforms:\n");
    for(unsigned int idNode = 0; idNode< points.size(); idNode++)
    {
        //fprintf(fout, "nodeJoint%d ", idNode);
        fprintf(fout, "%s ", names[idNode].c_str());

    }

    // Los valores de los nodos
    fprintf(fout, "\n\n");
    for(unsigned int i = 0; i< points.size(); i++)
    {
        fprintf(fout, "<<%e, %e, %e>>; ", points[i].X(), points[i].Y(), points[i].Z() );
    }

    // Los puntos y los pesos asociados.
    fprintf(fout, "\n\nPoint positions and weights:\n");
    assert(meshWeights.size() == (unsigned int)mesh->vn);

    int vCount = 0;
    MyMesh::VertexIterator vi;
    for(vi = mesh->vert.begin(); vi!=mesh->vert.end(); ++vi )
    {
        fprintf(fout, "<<%e, %e, %e>>; ", (*vi).P().X(), (*vi).P().Y(), (*vi).P().Z() );

        for(unsigned int w = 0; w< meshWeights[vCount].size(); w++)
        {
            fprintf(fout, "%e ", meshWeights[vCount][w] );
        }
        vCount++;

        fprintf(fout, "\n" );
    }

    fclose(fout);
	*/
}

void saveWeightsToMayaByName(Geometry* mesh, vector< vector<float> >& meshWeights,vector<string>& names, string sFile)
{
	//TODO
	assert(false);

	/*
    FILE* fout;
    fout = fopen(sFile.c_str(), "w");

    // Cuantos nodos actuan?
    fprintf(fout, "Number of Transforms:\n");
    fprintf(fout, "%d\n\n", names.size());

    // Cuantos vertices tiene la malla?
    fprintf(fout, "Number of points:\n");
    fprintf(fout, "%d\n\n", mesh->vn);

    // Los nombres de los nodos
    fprintf(fout, "Transforms:\n");
    for(unsigned int idNode = 0; idNode< names.size(); idNode++)
    {
        //fprintf(fout, "nodeJoint%d ", idNode);
        fprintf(fout, "%s ", names[idNode].c_str());

    }

    // Los puntos y los pesos asociados.
    fprintf(fout, "\n\nPoint positions and weights:\n");
    assert(meshWeights.size() == (unsigned int)mesh->vn);

    int vCount = 0;
    MyMesh::VertexIterator vi;
    for(vi = mesh->vert.begin(); vi!=mesh->vert.end(); ++vi )
    {
        fprintf(fout, "<<%e, %e, %e>>; ", (*vi).P().X(), (*vi).P().Y(), (*vi).P().Z() );

        for(unsigned int w = 0; w< meshWeights[vCount].size(); w++)
        {
            fprintf(fout, "%e ", meshWeights[vCount][w] );
        }
        vCount++;

        fprintf(fout, "\n" );
    }

    fclose(fout);
	*/
}

