#include "HarmonicCoords.h"

#include <DataStructures/Modelo.h>

using namespace vcg;

void deformMeshWithHC(MyMesh& mesh, MyMesh& cage, MyMesh& newMesh, MyMesh& newCage, std::vector< std::vector<float> >& PerVertHC)
{
    MyMesh::VertexIterator pt;
    MyMesh::VertexIterator ptOriginal;

    //float minV = 100;
    //float maxV = -100;

    ptOriginal = mesh.vert.begin();
    for(pt = newMesh.vert.begin(); pt!=newMesh.vert.end(); ++pt )
    {
        vcg::Point3d ptAux(0,0,0);

        MyMesh::VertexIterator cagePt;
        MyMesh::VertexIterator cagePtOrig = cage.vert.begin();

        for(cagePt = newCage.vert.begin(); cagePt!=newCage.vert.end(); )
        {
            ptAux += (cagePt->P()-cagePtOrig->P())*PerVertHC[pt->IMark()][cagePt->IMark()];
            //coordVertSum += PerVertHC[pt->IMark()][cagePt->IMark()];

            //minV = min(minV, PerVertHC[pt->IMark()][cagePt->IMark()]);
            //maxV = max(maxV, PerVertHC[pt->IMark()][cagePt->IMark()]);

            cagePtOrig++;
            cagePt++;
        }

        pt->P() = ptOriginal->P() + ptAux;
        ptOriginal++;
    }

   // printf("Suma para de coord esta entre el %f y el %f\n", minV, maxV); fflush(0);
}

void setHarmonicCoordsToModel(grid3d& grid, MyMesh& mesh, std::vector< std::vector<float> > &PerVertHC)
{
	/*
    PerVertHC.resize(mesh.vn);

    float minValue = 9, maxValue = -1;
    int over0Values = 0;
    int over0point5Values = 0;
    for(MyMesh::VertexIterator vi = mesh.vert.begin(); vi!=mesh.vert.end(); ++vi )
    {
        // Obtenemos la celda en la que cae el vertice
        Point3i cell = grid.cellId(vi->P());

        // Recorremos todos los pesos almacenados en la celda y las asignamos al vertice.
        PerVertHC[vi->IMark()].resize(grid.weightSize,0);
        for(int i = 0; i< grid.weightSize; i++)
        {
            PerVertHC[vi->IMark()][i] = grid.cells[cell.X()][cell.Y()][cell.Z()]->data->weights[i];

            minValue = min(minValue, grid.cells[cell.X()][cell.Y()][cell.Z()]->data->weights[i]);
            maxValue = max(maxValue, grid.cells[cell.X()][cell.Y()][cell.Z()]->data->weights[i]);

            if(grid.cells[cell.X()][cell.Y()][cell.Z()]->data->weights[i] > 0)
                over0Values++;

            if(grid.cells[cell.X()][cell.Y()][cell.Z()]->data->weights[i] > 0.5)
                over0point5Values++;
        }
    }

    printf("\nMin:%f - Max:%f, Over0:%d - Over0.5:%d", minValue, maxValue, over0Values, over0point5Values); fflush(0);

	*/
}

void loadHarmonicCoordinates(MyMesh& mesh, grid3d& grid, std::vector< std::vector<float> > &PerVertHC, string sSavingFile)
{
    grid.LoadGridFromFile(sSavingFile.c_str());
    setHarmonicCoordsToModel(grid, mesh, PerVertHC);
}

void getHarmonicCoordinates(MyMesh& mesh, MyMesh& cage, grid3d& grid, std::vector< std::vector<float> > &PerVertHC, unsigned int resolution, string sSavingFile)
{
    // Agrandamos un 1% la caja contenedora
    Box3d newBound = cage.bbox;
    double plusX = newBound.DimX()*0.005;
    double plusY = newBound.DimY()*0.005;
    double plusZ = newBound.DimZ()*0.005;

    newBound.min -= Point3d(plusX, plusY, plusZ);
    newBound.max += Point3d(plusX, plusY, plusZ);

    Point3i res(resolution, resolution, resolution);
    //Iniciamos el grid
    double cellSize = 0;
    if(newBound.DimX() > newBound.DimY())
    {
        if(newBound.DimX() > newBound.DimZ())
        {
            cellSize = newBound.DimX()/resolution;
            res.Y() = (int)ceil(newBound.DimY()/cellSize);
            res.Z() = (int)ceil(newBound.DimZ()/cellSize);

        }
        else
        {
            cellSize = newBound.DimZ()/resolution;
            res.Y() = (int)ceil(newBound.DimY()/cellSize);
            res.X() = (int)ceil(newBound.DimX()/cellSize);
        }
    }
    else
    {
        if(newBound.DimY() > newBound.DimZ())
        {
            cellSize = newBound.DimY()/resolution;
            res.X() = (int)ceil(newBound.DimX()/cellSize);
            res.Z() = (int)ceil(newBound.DimZ()/cellSize);
        }
        else
        {
            cellSize = newBound.DimZ()/resolution;
            res.Y() = (int)ceil(newBound.DimY()/cellSize);
            res.X() = (int)ceil(newBound.DimX()/cellSize);
        }
    }

    newBound.max = newBound.min + Point3d(res.X(), res.Y(), res.Z())*cellSize;
    grid = grid3d(newBound, res, cage.vn);

    //Tipificamos las celdas según si es interior, exterior, o borde de la maya
    int tipedCells = grid.typeCells(cage);

    //Extendemos los pesos en las celdas del interior
    grid.expandWeights();

    grid.normalizeWeights();

    setHarmonicCoordsToModel(grid, mesh, PerVertHC);

    if(!sSavingFile.empty())
        grid.SaveGridToFile(sSavingFile.c_str());

    printf("Fin del proceso!!!\n");fflush(0);

}


void updateBoundingBox(Modelo& m)
{
	double minx = 9999, miny = 9999, minz = 9999;
	double maxx = -9999,  maxy = -9999,  maxz = -9999;

	for(int i = 0; i< m.nodes.size(); i++)
	{
		minx = min(minx, m.nodes[i]->position.X());
		miny = min(miny, m.nodes[i]->position.Y());
		minz = min(minz, m.nodes[i]->position.Z());

		maxx = max(maxx, m.nodes[i]->position.X());
		maxy = max(maxy, m.nodes[i]->position.Y());
		maxz = max(maxz, m.nodes[i]->position.Z());
	}

	m.maxBBox = Point3d(maxx,maxy,maxz);
	m.minBBox = Point3d(minx,miny,minz);
}

void getHC_insideModel(Modelo& m, 
					   grid3d& grid, 
					   unsigned int resolution,
					   string sSavingFile,
					   float boxEnlarge// 1% de agandamiento
					   )
{
	// Obtenemos la caja envolvente para recoger el modelo

    // Agrandamos un 1% la caja contenedora
	updateBoundingBox(m);
	Box3d newBound(m.minBBox, m.maxBBox);

    double plusX = newBound.DimX()*boxEnlarge;
    double plusY = newBound.DimY()*boxEnlarge;
    double plusZ = newBound.DimZ()*boxEnlarge;

    newBound.min -= Point3d(plusX, plusY, plusZ);
    newBound.max += Point3d(plusX, plusY, plusZ);

    Point3i res(resolution, resolution, resolution);

    //Iniciamos el grid
    double cellSize = 0;
    if(newBound.DimX() > newBound.DimY())
    {
        if(newBound.DimX() > newBound.DimZ())
        {
            cellSize = newBound.DimX()/resolution;
            res.Y() = (int)ceil(newBound.DimY()/cellSize);
            res.Z() = (int)ceil(newBound.DimZ()/cellSize);

        }
        else
        {
            cellSize = newBound.DimZ()/resolution;
            res.Y() = (int)ceil(newBound.DimY()/cellSize);
            res.X() = (int)ceil(newBound.DimX()/cellSize);
        }
    }
    else
    {
        if(newBound.DimY() > newBound.DimZ())
        {
            cellSize = newBound.DimY()/resolution;
            res.X() = (int)ceil(newBound.DimX()/cellSize);
            res.Z() = (int)ceil(newBound.DimZ()/cellSize);
        }
        else
        {
            cellSize = newBound.DimZ()/resolution;
            res.Y() = (int)ceil(newBound.DimY()/cellSize);
            res.X() = (int)ceil(newBound.DimX()/cellSize);
        }
    }

	printf("BoundingBox: (%f,%f,%f) -> (%f,%f,%f)\n",newBound.min.X(),newBound.min.Y(),newBound.min.Z(),
													newBound.max.X(),newBound.max.Y(),newBound.max.Z());
	fflush(0);

    newBound.max = newBound.min + Point3d(res.X(), res.Y(), res.Z())*cellSize;
	grid = grid3d(newBound, res, m.nodes.size());

    //Tipificamos las celdas según si es interior, exterior, o borde de la maya
    printf("Typing cells\n"); fflush(0);
	int tipedCells = grid.typeCells(&m);

    //Extendemos los pesos en las celdas del interior
    //grid.expandWeights();
	printf("Expanding weights\n"); fflush(0);
	grid.expandWeightsOptimized(&m);

    //grid.normalizeWeights();
	printf("Normalizing weights\n"); fflush(0);
	grid.normalizeWeightsOptimized();

    //setHarmonicCoordsToModel(grid, mesh, PerVertHC);

	printf("Saving\n"); fflush(0);
    if(!sSavingFile.empty())
        grid.SaveGridToFile(sSavingFile.c_str());

    printf("Fin del proceso!!!\n");fflush(0);

}
