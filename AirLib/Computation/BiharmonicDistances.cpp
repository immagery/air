#include "BiharmonicDistances.h"

#define DEBUG_LOGGING false
//#include <QString>

using Eigen::MatrixXd;
using Eigen::Matrix3f;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::Vector3f;
using Eigen::Vector3d;

#include <utils/util.h>
#include "DataStructures/Modelo.h"
#include "DataStructures/SurfaceData.h"

int bindingBD(Modelo& modelo, binding* bd, std::vector<int>& indices, symMatrix& dists, bool withPatches)
{
	/*
	printf("computing Biharmonic distances.\n");
	fflush(0);

	clock_t begin, end;
	begin = clock();

	vector<MyFace*> bindingFaces;
	bindingFaces.resize(modelo.fn);
	MyMesh::FaceIterator fi;
	unsigned int faceCounter = 0;
    for(fi = modelo.face.begin(); fi!=modelo.face.end(); ++fi ) 
	{
		bool fromBinding = false;
		for(int fcvert = 0; fcvert < 3 && !fromBinding; fcvert++)
		{
			int idVert = (*fi).V(fcvert)->IMark();
			fromBinding |= (modelo.modelVertexBind[idVert] == bd->bindId);
		}
		
		if(fromBinding)
		{
			bindingFaces[faceCounter] = &(*fi);
			//bindingFaces.push_back(&(*fi));
			faceCounter++;
		}
	}
	bindingFaces.resize(faceCounter);

	int nopts = bd->pointData.size(); // Numero de vertices del modelo
	int notrg = bd->ntriangles = bindingFaces.size(); // Numero de tri‡ngulos del modelo

	if(withPatches) notrg += bd->virtualTriangles.size();

    assert(nopts > 0);
    assert(notrg > 0);


	printf("selection done.\n"); fflush(0);

    //////////////////////////////////////////////////////
    // CALCULO DEL AREA PARA CADA TRIANGULO DEL MODELO  //
    //////////////////////////////////////////////////////

    Vector3d bir(1,1,1);
    VectorXd trgarea(notrg); trgarea << VectorXd::Zero(notrg);

    int idx = 0;
    //for(fi = modelo.face.begin(); fi!=modelo.face.end(); ++fi ) 
	for(int findex = 0; findex < bindingFaces.size(); findex++ ) 
	{
		MyFace* fi = bindingFaces[findex];

        // Calculamos el area
        Vector3d trgx, trgy, trgz;
        trgx << (*fi).P(0).X(), (*fi).P(1).X(), (*fi).P(2).X(); // Los valores de X
        trgy << (*fi).P(0).Y(), (*fi).P(1).Y(), (*fi).P(2).Y(); // Los valores de Y
        trgz << (*fi).P(0).Z(), (*fi).P(1).Z(), (*fi).P(2).Z(); // Los valores de Z

        Matrix3d aa, bb, cc;
        aa << trgx, trgy, bir;
        bb << trgx, trgz, bir;
        cc << trgy, trgz, bir;

        double area = sqrt(pow(aa.determinant(),2)+pow(bb.determinant(),2)+pow(cc.determinant(),2))/2;
        trgarea(idx) = area;
        fi->IMark() = idx;

        idx++;
    }

	if(withPatches) 
	{
		for(int findex = 0; findex < bd->virtualTriangles.size(); findex++ ) 
		{
			Vector3d trgx, trgy, trgz;
			Point3d& pt1 = bd->virtualTriangles[findex].pts[0]->position;
			Point3d& pt2 = bd->virtualTriangles[findex].pts[1]->position;
			Point3d& pt3 = bd->virtualTriangles[findex].pts[2]->position;

			trgx << pt1.X(), pt2.X(), pt3.X(); // Los valores de X
			trgy << pt1.Y(), pt2.Y(), pt3.Y(); // Los valores de Y
			trgz << pt1.Z(), pt2.Z(), pt3.Z(); // Los valores de Z

			Matrix3d aa, bb, cc;
			aa << trgx, trgy, bir;
			bb << trgx, trgz, bir;
			cc << trgy, trgz, bir;

			double area = sqrt(pow(aa.determinant(),2)+pow(bb.determinant(),2)+pow(cc.determinant(),2))/2;
			trgarea(idx) = area;

			idx++;
		}
	}

	end = clock();
	printf("Triangle area computation: %f segs.\n", timelapse(end,begin)); fflush(0);
	begin = clock();

    if(DEBUG_LOGGING)
    {
        FILE* fout;
        fout = fopen("I:/Chus/phd_dev/Data/models/007_harry_files/001ownAreasTriangulos.txt", "w");
        fprintf(fout, "AreasTRG:\n");
        for(int i = 0; i< trgarea.rows(); i++)
        {
            fprintf(fout, "%g\n", trgarea(i));
        }
        fclose(fout);
    }

    //////////////////////////////////////////////////////
    // find the approximate voronoi area of each vertex //
    //////////////////////////////////////////////////////

    VectorXd AM(nopts);
    AM << VectorXd::Zero(nopts);
	*/

    // Recorremos los vertices y acumulamos el area de sus triangulos ponderado por 1/3 que le corresponde.
    /*
	MyMesh::VertexIterator vi;  idx = 0;
    for(vi = modelo.vert.begin(); vi!=modelo.vert.end(); ++vi ) 
	{
		int idVert = vi->IMark();
		if(modelo.modelVertexBind[idVert] != bd->bindId)
			continue;

         vcg::face::VFIterator<MyFace> vfi(&(*vi)); //initialize the iterator to the first face
		 for(;!vfi.End();++vfi) 
		 {       
			MyFace* f = vfi.F();
			int faceIdx = f->IMark();
            AM(idx) += trgarea(f->IMark())/3;
         }
		 idx++;
    }
	*/

	/*
	MyMesh::VertexIterator vi;  idx = 0;
	for(int vi = 0; vi < modelo.triangles.size(); vi++) 
	{
		bool binded = false;
		for(int iTemp = 0; iTemp < modelo.triangles[vi]->verts.size() && !binded; iTemp++ )
			binded |= modelo.modelVertexBind[modelo.triangles[vi]->verts[iTemp]->id] = bd->bindId;

		if(!binded) continue;

		for(int iTemp = 0; iTemp < modelo.triangles[vi]->verts.size() && !binded; iTemp++ )
		{
			AM(idx) += trgarea(vi)/3;
		}
		

         vcg::face::VFIterator<MyFace> vfi(&(*vi)); //initialize the iterator to the first face
		 for(;!vfi.End();++vfi) 
		 {       
			MyFace* f = vfi.F();
			int faceIdx = f->IMark();
            AM(idx) += trgarea(f->IMark())/3;
         }
		 idx++;
    }

	if(withPatches) 
	{
		for(int findex = 0; findex < bd->virtualTriangles.size(); findex++ ) 
		{
			Vector3d trgx, trgy, trgz;
			Point3d& pt1 = bd->virtualTriangles[findex].pts[0]->position;
			Point3d& pt2 = bd->virtualTriangles[findex].pts[1]->position;
			Point3d& pt3 = bd->virtualTriangles[findex].pts[2]->position;

			AM(bd->virtualTriangles[findex].pts[0]->modelVert) += trgarea(findex+bindingFaces.size())/3;
			AM(bd->virtualTriangles[findex].pts[1]->modelVert) += trgarea(findex+bindingFaces.size())/3;
			AM(bd->virtualTriangles[findex].pts[2]->modelVert) += trgarea(findex+bindingFaces.size())/3;

			idx++;
		}
	}

	printf("Intermedio.\n"); fflush(0);

    // now construct the cotan laplacian
    Eigen::SparseMatrix<double> A(nopts, nopts);
    //for(fi = modelo.face.begin(); fi!=modelo.face.end(); ++fi )
    for(int findex = 0; findex < bindingFaces.size(); findex++ ) 
	{
		MyFace* fi = bindingFaces[findex];

        for(int ii = 1; ii <= 3; ii++)
        {
            for(int jj = (ii+1); jj <= 3; jj++)
            {
                int kk = 6 - ii -jj;

                // indices reales
                int i = ii-1;
                int j = jj-1;
                int k = kk-1;

                //Vector3d e1; e1 << (*fi).P(i).X()-(*fi).P(j).X(), (*fi).P(i).Y()-(*fi).P(j).Y(), (*fi).P(i).Z()-(*fi).P(j).Z();
                Vector3d e2; e2 << (*fi).P(j).X()-(*fi).P(k).X(), (*fi).P(j).Y()-(*fi).P(k).Y(), (*fi).P(j).Z()-(*fi).P(k).Z();
                Vector3d e3; e3 << (*fi).P(i).X()-(*fi).P(k).X(), (*fi).P(i).Y()-(*fi).P(k).Y(), (*fi).P(i).Z()-(*fi).P(k).Z();

                double cosa = e2.dot(e3) / sqrt(e2.squaredNorm()*e3.squaredNorm());
                double sina = sqrt(1-pow(cosa,2));
                double cota = cosa/sina;
                double w = 0.5*cota;

                int v1 = modelo.modelVertexDataPoint[(*fi).V(i)->IMark()];
                int v2 = modelo.modelVertexDataPoint[(*fi).V(j)->IMark()];

                A.coeffRef(v1,v1) -= w;
                A.coeffRef(v1,v2) += w;
                A.coeffRef(v2,v2) -= w;
                A.coeffRef(v2,v1) += w;

             }
        }

		if(findex% 1000 == 0)	
			printf("Face %d.\n", findex); fflush(0);
    }

	if(withPatches) 
	{
		for(int findex = 0; findex < bd->virtualTriangles.size(); findex++ ) 
		{
			Vector3d trgx, trgy, trgz;
			Point3d pts[3];
			pts[0] = bd->virtualTriangles[findex].pts[0]->position;
			pts[1] = bd->virtualTriangles[findex].pts[1]->position;
			pts[2] = bd->virtualTriangles[findex].pts[2]->position;

			for(int ii = 1; ii <= 3; ii++)
			{
				for(int jj = (ii+1); jj <= 3; jj++)
				{
					int kk = 6 - ii -jj;

					// indices reales
					int i = ii-1;
					int j = jj-1;
					int k = kk-1;

					Vector3d e2; e2 << pts[j].X()-pts[k].X(), pts[j].Y()-pts[k].Y(), pts[j].Z()-pts[k].Z();
					Vector3d e3; e3 << pts[i].X()-pts[k].X(), pts[i].Y()-pts[k].Y(), pts[i].Z()-pts[k].Z();

					double cosa = e2.dot(e3) / sqrt(e2.squaredNorm()*e3.squaredNorm());
					double sina = sqrt(1-pow(cosa,2));
					double cota = cosa/sina;
					double w = 0.5*cota;

					int v1 = modelo.modelVertexDataPoint[bd->virtualTriangles[findex].Idxs[i]];
					int v2 = modelo.modelVertexDataPoint[bd->virtualTriangles[findex].Idxs[j]];

					A.coeffRef(v1,v1) -= w;
					A.coeffRef(v1,v2) += w;
					A.coeffRef(v2,v2) -= w;
					A.coeffRef(v2,v1) += w;
				 }
			}
		}
	}

	end = clock();
	printf("Cotangents: %f segs.\n", timelapse(end,begin)); fflush(0);
	begin = clock();

    if(DEBUG_LOGGING)
    {
        FILE* fout;
        fout = fopen("I:/Chus/phd_dev/Data/models/007_harry_files/002ownMatrizA.txt", "w");
        fprintf(fout, "A:\n");
        for(int i = 0; i< nopts; i++)
        {
            for(int j = 0; j < nopts; j++)
            {
                fprintf(fout, "(%d,%d) %5.10e\n",i+1,j+1, A.coeffRef(i,j));
            }
        }
        fclose(fout);
    }

    A *= -1;
    Eigen::SparseMatrix<double> T(nopts, nopts);
    for(int i = 0; i< nopts; i++)
    {
        assert(AM(i) != 0);
        double v = 1.0/AM(i);
        T.coeffRef(i,i) = v;
    }
    A = A*T*A;

	printf("Intermedio 2.\n", timelapse(end,begin)); fflush(0);

    for(int i = 0; i< nopts; i++)
        A.coeffRef(i,0) = 0.0;

    for(int j = 0; j < nopts; j++)
        A.coeffRef(0,j) = 0.0;

    A.coeffRef(0,0) = 1.0;

    if(DEBUG_LOGGING)
    {
        FILE* fout;
        fout = fopen("I:/Chus/phd_dev/Data/models/007_harry_files/003ownAntesDeResolver.txt", "w");
        fprintf(fout, "A antes de resolver:\n");
        for(int i = 0; i< nopts; i++)
        {
            for(int j = 0; j < nopts; j++)
            {
                fprintf(fout, "(%d,%d) %5.10e\n",i+1,j+1, A.coeffRef(i,j));
            }
        }
        fclose(fout);
    }

    /////////////////////////////
    //   SOLVE WITH SUPERLU    //
    /////////////////////////////
	dists.resize(nopts);
	//MatrixXd G(nopts,indices.size());
	VectorXd Gdiag(nopts);
	
	for(int ddd = 0; ddd < Gdiag.size(); ddd++)
		Gdiag[ddd] = 0;

    VectorXd Gaux(nopts);
    VectorXd H(nopts); // make the right hand side vector(s) for the system

	end = clock();
	printf("Matrix preprocess adjustments: %f segs.\n", timelapse(end,begin)); fflush(0);
	begin = clock();

    // Preprocesamos la matriz para acelerar los calculos LU
	Eigen::SuperLU<Eigen::SparseMatrix<double> > slu;
    slu.compute(A);

	end = clock();
	printf("Precalculo de A: %f segs.\n", timelapse(end,begin)); fflush(0);
	begin = clock();

    if(DEBUG_LOGGING)
    {
        MatrixXd H2(nopts,indices.size());
        for(unsigned int j = 0; j< indices.size(); j++)
        {
            //int idx = indices[j];
            // Solo creamos el vector que necesitamos para el c‡lculo.
            for( int i = 0; i< nopts; i++)
            {
                if(i == 0)
                {
                    H2(i,j) = 0; // No es necesario, de momento lo dejo por claridad
                }
                else if((int)j == i)
                {
                    double v = 1.0-(1.0/nopts);
                    H2(i,j) = v;
                }
                else
                {
                    double v = -(1.0/nopts);
                    H2(i,j) = v;
                }
            }
        }

        FILE* fout;
        fout = fopen("I:/Chus/phd_dev/Data/models/007_harry_files/004ownH.txt", "w");
        fprintf(fout, "H artificial:\n");
        for(int i = 0; i< nopts; i++)
        {
            for(unsigned int j = 0; j < indices.size(); j++)
            {
                fprintf(fout, "(%d,%d) %5.10e\n", i+1, j+1, H2(i,j));
            }
            fprintf(fout, "\n");
        }
        fclose(fout);
    }

    double diagonalValue = 1.0-(1.0/nopts);
    double restValue = -(1.0/nopts);
	int percent = 0;
    for(unsigned int col = 0; col< indices.size(); col++)
    {
        Gaux.setZero(nopts);
        int idx = indices[col];

        // Solo creamos el vector que necesitamos para el c‡lculo.
        for(int row = 0; row< nopts; row++)
        {
            if(row == 0)
                H(row) = (double)0.0; // No es necesario, de momento lo dejo por claridad
            else if(row == idx)
            {
                double v = diagonalValue; H(row) = v;
            }
            else
            {
                double v = restValue; H(row) = v;
            }
        }

        Gaux = slu.solve(H); // Solucionamos el sistema de ecuaciones

        //NORMALIZACION -> centramos segœn la media.
        //shift the solution so that add up to zero, but weighted
        double media = 0;
        for(int row = 0; row< nopts; row++)
		{
            // Pongo i porque la matriz es del tama–o indices.
            //G(j,i) = Gaux(j);
			dists.set(row,col, Gaux(row));
			
			if(row == col) 
				Gdiag(row) = Gaux(row);

			media += Gaux(row);
        }

        media = media / nopts;
		
        for(int row = 0; row< nopts; row++)
        {
            // Pongo i porque la matriz es del tamano indices.
            //G(j,i) -= media;
			double newValue = dists.get(row,col) - media;
			dists.set(row,col, newValue);

			if(row == col) 
				Gdiag(row) = Gdiag(row) - media;
		}

		if(col/200 != percent)
		{
			percent = col/200;
			printf("Porcentaje %f\n", (float)col/(float)indices.size()*100.0); fflush(0);
		}
    }

	end = clock();
	printf("Resolucion del sistema: %f segs.\n", timelapse(end,begin)); fflush(0);
	begin = clock();

    if(DEBUG_LOGGING)
    {
        FILE* fout;
        fout = fopen("I:/Chus/phd_dev/Data/models/007_harry_files/005ownGdpResolver.txt", "w");
        fprintf(fout, "G despues de resolver:\n");
        for(int i = 0; i< nopts; i++)
        {
            for(int j = 0; j < nopts; j++)
            {
                fprintf(fout, "(%d,%d) %5.10e\n", i+1, j+1, dists.get(i,j));
            }
            fprintf(fout, "\n");
        }
        fclose(fout);
    }

    //compute dists; this is a vectorization of d^2 = g(i,i)+g(j,j)-2g(i,j)
    for(int row = 0; row< nopts; row++)
    {
        for(int col = row; col< nopts; col++)
        {
            //double value = sqrt(G(i,i)+G(j,j)-2*G(i,j));

			// Quito el caso de la diagonal porque se anulan las componentes.
			//double diag1 = Gdiag(row);
			//double diag2 = Gdiag(col);
			//double distValue = Gdiag(row)+Gdiag(col)-2*dists.get(row,col);
			double value = sqrt(Gdiag(row)+Gdiag(col)-2*dists.get(row,col));
            dists.set(row,col,value);
            //dists(j,i) = value;
        }
    }

	end = clock();
	printf("Guardar valores(podriamos considerarla simetrica): %f segs.\n", timelapse(end,begin)); fflush(0);
	begin = clock();

	*/
	return 0;
}

int biharmonicDistances(MyMesh& modelo, std::vector<int>& indices, Eigen::MatrixXd& dists)
{
	printf("computing Biharmonic distances.\n");
	fflush(0);

	clock_t begin, end;
	begin = clock();

	int nopts = modelo.vn; // Numero de vertices del modelo
    int notrg = modelo.fn; // Numero de tri‡ngulos del modelo

    assert(nopts > 0);
    assert(notrg > 0);

    //////////////////////////////////////////////////////
    // CALCULO DEL AREA PARA CADA TRIANGULO DEL MODELO  //
    //////////////////////////////////////////////////////

    Vector3d bir(1,1,1);
    VectorXd trgarea(notrg); trgarea << VectorXd::Zero(notrg);

    int idx = 0;
    MyMesh::FaceIterator fi;
    for(fi = modelo.face.begin(); fi!=modelo.face.end(); ++fi ) {

        // Calculamos el area
        Vector3d trgx, trgy, trgz;
        trgx << (*fi).P(0).X(), (*fi).P(1).X(), (*fi).P(2).X(); // Los valores de X
        trgy << (*fi).P(0).Y(), (*fi).P(1).Y(), (*fi).P(2).Y(); // Los valores de Y
        trgz << (*fi).P(0).Z(), (*fi).P(1).Z(), (*fi).P(2).Z(); // Los valores de Z

        Matrix3d aa, bb, cc;
        aa << trgx, trgy, bir;
        bb << trgx, trgz, bir;
        cc << trgy, trgz, bir;

        double area = sqrt(pow(aa.determinant(),2)+pow(bb.determinant(),2)+pow(cc.determinant(),2))/2;
        trgarea(idx) = area;
        fi->IMark() = idx;

        idx++;
     }

	end = clock();
	printf("Triangle area computation: %f segs.\n", timelapse(end,begin)); fflush(0);
	begin = clock();

    if(DEBUG_LOGGING)
    {
        FILE* fout;
        fout = fopen("/Users/chus/proyectos/phD/phd_dev/Data/001ownAreasTriangulos.txt", "w");
        fprintf(fout, "AreasTRG:\n");
        for(int i = 0; i< trgarea.rows(); i++)
        {
            fprintf(fout, "%g\n", trgarea(i));
        }
        fclose(fout);
    }

    //////////////////////////////////////////////////////
    // find the approximate voronoi area of each vertex //
    //////////////////////////////////////////////////////

    VectorXd AM(nopts);
    AM << VectorXd::Zero(nopts);

    // Recorremos los vŽrtices y acumulamos el area de sus tri‡ngulos ponderado por 1/3 que le corresponde.
    MyMesh::VertexIterator vi;  idx = 0;
    for(vi = modelo.vert.begin(); vi!=modelo.vert.end(); ++vi ) {
         vcg::face::VFIterator<MyFace> vfi(&(*vi)); //initialize the iterator to the first face
         for(;!vfi.End();++vfi) {
           MyFace* f = vfi.F();
           AM(idx) += trgarea(f->IMark())/3;
         }
         idx++;
     }

    // now construct the cotan laplacian
    Eigen::SparseMatrix<double> A(nopts, nopts);
    for(fi = modelo.face.begin(); fi!=modelo.face.end(); ++fi )
    {
        for(int ii = 1; ii <= 3; ii++)
        {
            for(int jj = (ii+1); jj <= 3; jj++)
            {
                int kk = 6 - ii -jj;

                // indices reales
                int i = ii-1;
                int j = jj-1;
                int k = kk-1;

                //Vector3d e1; e1 << (*fi).P(i).X()-(*fi).P(j).X(), (*fi).P(i).Y()-(*fi).P(j).Y(), (*fi).P(i).Z()-(*fi).P(j).Z();
                Vector3d e2; e2 << (*fi).P(j).X()-(*fi).P(k).X(), (*fi).P(j).Y()-(*fi).P(k).Y(), (*fi).P(j).Z()-(*fi).P(k).Z();
                Vector3d e3; e3 << (*fi).P(i).X()-(*fi).P(k).X(), (*fi).P(i).Y()-(*fi).P(k).Y(), (*fi).P(i).Z()-(*fi).P(k).Z();

                double cosa = e2.dot(e3) / sqrt(e2.squaredNorm()*e3.squaredNorm());
                double sina = sqrt(1-pow(cosa,2));
                double cota = cosa/sina;
                double w = 0.5*cota;

                int v1 = (*fi).V(i)->IMark();
                int v2 = (*fi).V(j)->IMark();

                A.coeffRef(v1,v1) -= w;
                A.coeffRef(v1,v2) += w;
                A.coeffRef(v2,v2) -= w;
                A.coeffRef(v2,v1) += w;

             }
        }
    }

	end = clock();
	printf("Cotangents: %f segs.\n", timelapse(end,begin)); fflush(0);
	begin = clock();

    if(DEBUG_LOGGING)
    {
        FILE* fout;
        fout = fopen("/Users/chus/proyectos/phD/phd_dev/Data/002ownMatrizA.txt", "w");
        fprintf(fout, "A:\n");
        for(int i = 0; i< nopts; i++)
        {
            for(int j = 0; j < nopts; j++)
            {
                fprintf(fout, "(%d,%d) %5.10e\n",i+1,j+1, A.coeffRef(i,j));
            }
        }
        fclose(fout);
    }

    A *= -1;
    Eigen::SparseMatrix<double> T(nopts, nopts);
    for(int i = 0; i< nopts; i++)
    {
        assert(AM(i) != 0);
        double v = 1.0/AM(i);
        T.coeffRef(i,i) = v;
    }
    A = A*T*A;

    for(int i = 0; i< nopts; i++)
        A.coeffRef(i,0) = 0.0;

    for(int j = 0; j < nopts; j++)
        A.coeffRef(0,j) = 0.0;

    A.coeffRef(0,0) = 1.0;

    if(DEBUG_LOGGING)
    {
        FILE* fout;
        fout = fopen("/Users/chus/proyectos/phD/phd_dev/Data/003ownAntesDeResolver.txt", "w");
        fprintf(fout, "A antes de resolver:\n");
        for(int i = 0; i< nopts; i++)
        {
            for(int j = 0; j < nopts; j++)
            {
                fprintf(fout, "(%d,%d) %5.10e\n",i+1,j+1, A.coeffRef(i,j));
            }
        }
        fclose(fout);
    }

    /////////////////////////////
    //   SOLVE WITH SUPERLU    //
    /////////////////////////////
    MatrixXd G(nopts,indices.size());
    VectorXd Gaux(nopts);
    VectorXd H(nopts); // make the right hand side vector(s) for the system

	end = clock();
	printf("Matrix preprocess adjustments: %f segs.\n", timelapse(end,begin)); fflush(0);
	begin = clock();

    // Preprocesamos la matriz para acelerar los calculos LU
	Eigen::SuperLU<Eigen::SparseMatrix<double> > slu;
    slu.compute(A);

	end = clock();
	printf("Precalculo de A: %f segs.\n", timelapse(end,begin)); fflush(0);
	begin = clock();

    if(DEBUG_LOGGING)
    {
        MatrixXd H2(nopts,indices.size());
        for(unsigned int j = 0; j< indices.size(); j++)
        {
            //int idx = indices[j];
            // Solo creamos el vector que necesitamos para el c‡lculo.
            for( int i = 0; i< nopts; i++)
            {
                if(i == 0)
                {
                    H2(i,j) = 0; // No es necesario, de momento lo dejo por claridad
                }
                else if((int)j == i)
                {
                    double v = 1.0-(1.0/nopts);
                    H2(i,j) = v;
                }
                else
                {
                    double v = -(1.0/nopts);
                    H2(i,j) = v;
                }
            }
        }

        FILE* fout;
        fout = fopen("/Users/chus/proyectos/phD/phd_dev/Data/004ownH.txt", "w");
        fprintf(fout, "H artificial:\n");
        for(int i = 0; i< nopts; i++)
        {
            for(unsigned int j = 0; j < indices.size(); j++)
            {
                fprintf(fout, "(%d,%d) %5.10e\n", i+1, j+1, H2(i,j));
            }
            fprintf(fout, "\n");
        }
        fclose(fout);
    }

    double diagonalValue = 1.0-(1.0/nopts);
    double restValue = -(1.0/nopts);
	int percent = 0;
    for(unsigned int i = 0; i< indices.size(); i++)
    {
        Gaux.setZero(nopts);
        int idx = indices[i];
        // Solo creamos el vector que necesitamos para el c‡lculo.
        for(int j = 0; j< nopts; j++)
        {
            if(j == 0)
                H(j) = (double)0.0; // No es necesario, de momento lo dejo por claridad
            else if(j == idx)
            {
                double v = diagonalValue; H(j) = v;
            }
            else
            {
                double v = restValue; H(j) = v;
            }
        }

        Gaux = slu.solve(H); // Solucionamos el sistema de ecuaciones

        //NORMALIZACION -> centramos segœn la media.
        //shift the solution so that add up to zero, but weighted
        double media = 0;
        for(int j = 0; j< nopts; j++)
        {
            // Pongo i porque la matriz es del tama–o indices.
            G(j,i) = Gaux(j);
            media += Gaux(j);
        }

        media = media / nopts;

        for(int j = 0; j< nopts; j++)
        {
            // Pongo i porque la matriz es del tama–o indices.
            G(j,i) -= media;
        }

		if(i/100 != percent)
		{
			percent = i/100;
			printf("Porcentaje %f\n", (float)i/(float)indices.size()); fflush(0);
		}
    }

	end = clock();
	printf("Resolucion del sistema: %f segs.\n", timelapse(end,begin)); fflush(0);
	begin = clock();

    if(DEBUG_LOGGING)
    {
        FILE* fout;
        fout = fopen("/Users/chus/proyectos/phD/phd_dev/Data/005ownGdpResolver.txt", "w");
        fprintf(fout, "G despues de resolver:\n");
        for(int i = 0; i< nopts; i++)
        {
            for(int j = 0; j < nopts; j++)
            {
                fprintf(fout, "(%d,%d) %5.10e\n", i+1, j+1, G(i,j));
            }
            fprintf(fout, "\n");
        }
        fclose(fout);
    }

    //compute dists; this is a vectorization of d^2 = g(i,i)+g(j,j)-2g(i,j)
    int idxSize = indices.size();
    dists.resize(idxSize,idxSize);
    for(int i = 0; i< idxSize; i++)
    {
        for(int j = i; j< idxSize; j++)
        {
            double value = sqrt(G(i,i)+G(j,j)-2*G(i,j));
            dists(i,j) = value;
            dists(j,i) = value;
        }
    }

	end = clock();
	printf("Guardar valores(podriamos considerarla simetrica): %f segs.\n", timelapse(end,begin)); fflush(0);
	begin = clock();

    return idxSize;
}


