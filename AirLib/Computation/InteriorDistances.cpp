#include "InteriorDistances.h"
#include "mvc_interiorDistances.cpp"

#include <iostream>
#include <fstream>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <QtCore/QTime>
#include <QtCore/QFile>
#include <QtCore/QTextStream>

#include "../ArpackPlusLib/arpackpluslib.h"

int getSparseTA(MyMesh& modelo, Eigen::SparseMatrix<double>& T, Eigen::SparseMatrix<double>& A)
{

    if(VERBOSE_ID)
    {
        printf("FUNCTION: getSparseTA -- DEBUG --\n"); fflush(0);
    }

    int nopts = modelo.vn; // Numero de vertices del modelo
    int notrg = modelo.fn; // Numero de tri‡ngulos del modelo

    //////////////////////////////////////////////////////
    // CALCULO DEL çREA PARA CADA TRIANGULO DEL MODELO  //
    //////////////////////////////////////////////////////

    Vector3d bir(1,1,1);
    VectorXd trgarea(notrg); trgarea << VectorXd::Zero(notrg);

    int idx = 0;
    MyMesh::FaceIterator fi;
    for(fi = modelo.face.begin(); fi!=modelo.face.end(); ++fi )
    {
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


    FILE* fout;

    if(VERBOSE_ID)
    {
        printf("Grabamos en disco los valores area de los triangulos\n"); fflush(0);

        fout = fopen("001areasTriangulos.txt", "w");
        for(int i = 0; i< trgarea.rows(); i++)
        {
            fprintf(fout,"%3.10e\n",trgarea(i));
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
    A.resize(nopts, nopts);
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

                Vector3d e1; e1 << (*fi).P(i).X()-(*fi).P(j).X(), (*fi).P(i).Y()-(*fi).P(j).Y(), (*fi).P(i).Z()-(*fi).P(j).Z();
                Vector3d e2; e2 << (*fi).P(j).X()-(*fi).P(k).X(), (*fi).P(j).Y()-(*fi).P(k).Y(), (*fi).P(j).Z()-(*fi).P(k).Z();
                Vector3d e3; e3 << (*fi).P(i).X()-(*fi).P(k).X(), (*fi).P(i).Y()-(*fi).P(k).Y(), (*fi).P(i).Z()-(*fi).P(k).Z();

                double cosa = e2.dot(e3) / sqrt(e2.squaredNorm()*e3.squaredNorm());
                double sina = sqrt(1-pow(cosa,2));
                double cota = cosa/sina;
                double w = 0.5*cota;

                int v1 = (*fi).V(i)->IMark();
                int v2 = (*fi).V(j)->IMark();

                if(w != 0)
                {
                    A.coeffRef(v1,v1) -= w;
                    A.coeffRef(v1,v2) += w;
                    A.coeffRef(v2,v2) -= w;
                    A.coeffRef(v2,v1) += w;
                }

           }
        }
    }
    A.makeCompressed();
	
    printf("Ojo, este trozo de funci—n es posible que no valga para biharmonic distances, es necesario hacer la inversa 1/X\n"); fflush(0);

    T.resize(nopts, nopts);
    for(int i = 0; i< nopts; i++)
    {
        assert(AM(i) != 0);
        T.coeffRef(i,i) = AM(i);
    }
    T.makeCompressed();

    if(VERBOSE_ID)
    {
        printf("Grabamos en disco los valores a T y A para test\n"); fflush(0);

        fout = fopen("getSparseTA_res_matrizA.txt", "w");
        for (int k=0; k<A.outerSize(); ++k)
        {
          for (Eigen::SparseMatrix<double>::InnerIterator it(A,k); it; ++it)
          {
              fprintf(fout,"(%d, %d) %3.10e\n",it.row()+1,it.col()+1,it.value());
          }
        }
        fclose(fout);

        fout = fopen("getSparseTA_res_matrizT.txt", "w");
        for (int k=0; k<T.outerSize(); ++k)
        {
          for (Eigen::SparseMatrix<double>::InnerIterator it(T,k); it; ++it)
          {
              fprintf(fout,"(%d, %d) %3.10e\n",it.row()+1,it.col()+1,it.value());
          }
        }
        fclose(fout);

        printf("Ya se ha volcado todo el contenido debug\n"); fflush(0);
    }

    return 0;
}


bool buildCSCSparseMatrix(int dim, sparseMatrix& auxMatrix, Eigen::SparseMatrix<double>& matrix)
{
    if(VERBOSE_ID)
    {
        printf("Funcion buildCSCSparseMatrix -- DEBUG --.\n"); fflush(0);
    }

    auxMatrix.val.resize(matrix.nonZeros(),0);
    auxMatrix.irow.resize(matrix.nonZeros(),0);
    auxMatrix.pcol.resize(dim+1,0);

    int colAnt = 0;
    int valIdx = 0;
    int colIdx = 1;

    auxMatrix.pcol[0] = 0;
    bool isSymmetric = true;

    for (int k=0; k<matrix.outerSize(); ++k)
    {
      for (Eigen::SparseMatrix<double>::InnerIterator it(matrix,k); it; ++it)
      {
        auxMatrix.val[valIdx] = it.value();
        auxMatrix.irow[valIdx] = it.row();

        if(VERBOSE_ID)
        {
            isSymmetric &= (matrix.coeffRef(it.row(), it.col()) == matrix.coeffRef(it.col(), it.row()));
            if(!isSymmetric)
            {
                printf("Ohhhhh!, hay un valor (%d, %d) por el que la matriz no es simetrica\n", it.row(), it.col());
                fflush(0);
            }
        }

        if(colAnt < it.col())
        {
            auxMatrix.pcol[colIdx] = valIdx;
            colAnt = it.col(); colIdx++;
        }
        valIdx++;
      }
    }

    if(VERBOSE_ID && isSymmetric)
    {
        printf("Perfecto!!!, la matriz es simetrica.\n"); fflush(0);
    }

    auxMatrix.pcol[dim] = valIdx;
    auxMatrix.nnz = valIdx;

    return true;
}


bool buildCSCSparseSymmetricMatrix(int dim, sparseMatrix& auxMatrix, Eigen::SparseMatrix<double>& matrix)
{
    if(VERBOSE_ID)
    {
        printf("Funcion buildCSCSparseSymmetricMatrix -- DEBUG --.\n"); fflush(0);
    }

    auxMatrix.val.resize(matrix.nonZeros(),0);
    auxMatrix.irow.resize(matrix.nonZeros(),0);
    auxMatrix.pcol.resize(dim+1,0);

    int colAnt = 0;
    int valIdx = 0;
    int colIdx = 1;

    auxMatrix.pcol[0] = 0;

    for (int k=0; k<matrix.outerSize(); ++k)
    {
      for (Eigen::SparseMatrix<double>::InnerIterator it(matrix,k); it; ++it)
      {
        auxMatrix.val[valIdx] = it.value();
        auxMatrix.irow[valIdx] = it.row();

        if(it.row() < it.col())
            continue;

        if(colAnt < it.col())
        {
            auxMatrix.pcol[colIdx] = valIdx;
            colAnt = it.col(); colIdx++;
        }
        valIdx++;
      }
    }

    auxMatrix.pcol[dim] = valIdx;
    auxMatrix.nnz = valIdx;

    auxMatrix.val.resize(valIdx,0);
    auxMatrix.irow.resize(valIdx,0);

    return true;
 }


int TestOverMatrixConstruction()
{
    /*
    printf("Creaci—n\n"); fflush(0);
    Eigen::SparseMatrix<double> pruebas(5,5);

    pruebas.coeffRef(1,0) = 22;
    pruebas.coeffRef(2,0) = 7;
    pruebas.coeffRef(0,1) = 3;
    pruebas.coeffRef(2,1) = 5;
    pruebas.coeffRef(4,2) = 14;
    pruebas.coeffRef(2,3) = 1;
    pruebas.coeffRef(1,4) = 17;
    pruebas.coeffRef(4,4) = 8;

    pruebas.makeCompressed();

    sparseMatrix pruebasAux;
    buildCSCSparseMatrix(5, pruebasAux, pruebas);

    printf("Pruebas de composici—n de matrices.\n");
    printf("Vals (%d) rows(%d) cols(%d)\n", pruebasAux.val.size(), pruebasAux.irow.size(), pruebasAux.pcol.size());
    printf("Valores: ");
    for(int i = 0; i< pruebasAux.val.size(); i++)
        printf("%f ", pruebasAux.val[i]);
    printf("\n");
    printf("Rows: ");
    for(int i = 0; i< pruebasAux.irow.size(); i++)
        printf("%d ", pruebasAux.irow[i]);
    printf("\n");
    printf("PCols: ");
    for(int i = 0; i< pruebasAux.pcol.size(); i++)
        printf("%d ", pruebasAux.pcol[i]);
    printf("\n");

    fflush(0);
    */
    /*
    Aaux.val.resize(A.nonZeros(),0);
    Aaux.irow.resize(A.nonZeros(),0);
    Aaux.pcol.resize(nopts+1,0);

    int colAnt = 0;
    int valIdx = 0;
    int rowIdx = 0;
    int colIdx = 1;

    Aaux.pcol[0] = 0;
    Aaux.nnz = nopts;

    for (int k=0; k<A.outerSize(); ++k)
    {
      for (Eigen::SparseMatrix<double>::InnerIterator it(A,k); it; ++it)
      {
        Aaux.val[valIdx] = it.value();
        Aaux.irow[rowIdx] = it.row();

        if(colAnt < it.col())
        {
            Aaux.pcol[colIdx] = it.col();
            colAnt = it.col(); colIdx++;
        }

        valIdx++;
        rowIdx++;
      }
    }

    Aaux.pcol[nopts] = nopts;

    Taux.nnz = nopts;
    Taux.val.resize(T.nonZeros());
    Taux.irow.resize(T.nonZeros());
    Taux.pcol.resize(nopts+1);

    colAnt = 0;
    valIdx = 0;
    colIdx = 1;
    Taux.pcol[0] = 0;
    for (int k=0; k<T.outerSize(); ++k)
    {
      for (Eigen::SparseMatrix<double>::InnerIterator it(T,k); it; ++it)
      {
        Taux.val[valIdx] = it.value();
        Taux.irow[valIdx] = it.row();

        if(colAnt != it.col())
        {
            Taux.pcol[colIdx] = it.col();
            colAnt = it.col(); colIdx++;
        }
        valIdx++;
      }
    }
    Taux.pcol[nopts] = nopts;
    */

    return 0;
}

int getEigenVectorSolution(MyMesh& modelo, vector<double>& eigenValuesRes, vector< vector<double> >& eigenVectorsRes)
{
    // two adjustable parameters:
    // t -- time parameter for diffusion
    double t = 0.125;
    // tol -- tolerance for eigenvec/eigenvalue cutoff, see appendix
    double tol = 0.000001;

    int nopts = modelo.vn; // Numero de vertices del modelo

    Eigen::SparseMatrix<double> T;
    Eigen::SparseMatrix<double> A;

    getSparseTA(modelo, T, A);

    int eigno = 20;
    double logtol = -log(tol);

    sparseMatrix Aaux;
    sparseMatrix Taux;

    buildCSCSparseSymmetricMatrix(nopts, Aaux, A);
    buildCSCSparseSymmetricMatrix(nopts, Taux, T);

    vector<double> eigenValues;
    vector< vector<double> > eigenVectors;

    int it = 0;
    if(VERBOSE_ID) { printf("Calculo de EigenValues and EigenVectors\n"); fflush(0); }

    bool devam = true;
    while(devam)
    {
        if(VERBOSE_ID)
        {
            if(eigno <= 0) {
                printf("Tenemos un problema HOUSTON (%d)\n", eigno);
                break;
            }
            printf("Iteracion %d -> Eigen values = %d\n", it, eigno); fflush(0);
        }

        // solucionar eigenvalues
        assert(true); //testing other things... you need to enable it.
        int noconv = 0/*eigs(nopts, Aaux, Taux, eigno, 0.0, eigenValues, eigenVectors, 1/pow(10.0,8))*/;

        if(VERBOSE_ID && noconv != 0) printf("ERROR!: No convergen todos los eig values.\n");

        double ratio = t*eigenValues[eigno-1]/eigenValues[1];

        if (ratio<logtol)
            eigno = ceil(eigno*logtol/ratio);
        else
            devam = false;

        if(VERBOSE_ID)
        {
            for(unsigned int i = 0; i< eigenValues.size(); i++)
            {
                printf("Value->(%f)\n", eigenValues[i]);
            }
            it++;
        }
    }

    if(VERBOSE_ID)
    {
        printf("Grabamos en disco los valores de V\n"); fflush(0);
        FILE *fout;
        fout = fopen("004eigenVectors_init.txt", "w");
        for(unsigned int i = 0; i< eigenVectors.size(); i++)
        {
            for(unsigned int j = 0; j< eigenVectors[i].size(); j++)
            {
                fprintf(fout,"(%d, %d) %3.10e\n",i+1,j+1, eigenVectors[i][j]);
            }
        }
        fclose(fout);
    }

    eigno--; // Descartamos el primer elemento
    eigenValuesRes.resize(eigno);

    ////////////////////////////////
    //TODO -> escalado de los valores de eigen vectors -> sino son demasiado pequeños,
    //da problemas de precision.
    assert(false);
    ///////////////////////////////

    // Copiamos los eigen values descartando el primero
    for(unsigned int i = 1; i< eigenValues.size(); i++)
        eigenValuesRes[i-1] = eigenValues[i]*(-1);


    // Copiamos los primeros eigeno valores de cada vector aplicando una transformaci—n
    // Descartamos tambiŽn el primer elemento
    assert(eigenValuesRes[0] != 0); // Aseguramos que no habr‡ divisi—n entre cero.
    eigenVectorsRes.resize(nopts);
    for(int i = 0; i< nopts; i++)
    {
        eigenVectorsRes[i].resize(eigno);
        for(int j = 0; j< eigno; j++)
        {
            eigenVectorsRes[i][j] = eigenVectors[i][j+1]*exp((-1)*t*eigenValuesRes[j]/eigenValuesRes[0]);
        }
    }

    if(VERBOSE_ID)
    {
        for(int j = 0; j< eigno; j++)
        {
            double sign = eigenVectorsRes[0][j]/fabs(eigenVectorsRes[0][j]);
            for(unsigned int i = 0; i< eigenVectorsRes.size(); i++)
            {
                eigenVectorsRes[i][j] = eigenVectorsRes[i][j]*sign;
            }
        }

        FILE *fout;
        fout = fopen("005eigenVectors.txt", "w");
        for(unsigned int i = 0; i< eigenVectorsRes.size(); i++)
        {
            for(unsigned int j = 0; j< eigenVectorsRes[i].size(); j++)
            {
                fprintf(fout,"%3.10e\n", eigenVectorsRes[i][j]);
            }
        }
        fclose(fout);
    }

    /*
    V = V(:,2:eigno); DD = -diag(D); DD = DD(2:eigno); eigno=eigno-1;

    DD2 = DD

    for i=1:eigno
        V(:,i) = V(:,i)*exp(-t*DD(i)/DD(1));
    end
    */

    return eigno;
}

int interiorDistancesCompleteTest(MyMesh& modelo, vector<vcg::Point3d>& points, vector<double>& distances, bool bVerbose)
{
    if(points.size() <= 0)
        return 0;

    vector<double> eigenValues;
    vector< vector<double> > eigenVectors;

    int eigno = getEigenVectorSolution( modelo, eigenValues, eigenVectors);

    if(bVerbose)
        printf("Se han calculado %d eigenValues\n", eigno);

    /*----------------------------------------//
         Compute the interior distance
    //----------------------------------------*/
    vector< vector<double> > embeddedPoints;
    int nPoints = points.size();

    embeddedPoints.resize(nPoints);

    // Embed coordinates.
    mvc_IBD(points, embeddedPoints, eigenVectors, modelo); // Embedding over every point to compute distance.

    if(bVerbose)
    {
        FILE* foutExtra;
        foutExtra = fopen("eigenVectorResultantes.txt","w");
        for(unsigned int i = 0; i<embeddedPoints.size(); i++)
        {
            for(unsigned int j = 0; j<embeddedPoints[i].size(); j++)
            {
                fprintf(foutExtra,"%3.10e\n",embeddedPoints[i][j]);
            }
        }
        fclose(foutExtra);
    }

    // Distance calculus with embeded coordinates.
    distances.resize(nPoints*(nPoints-1));
    int idDistance = 0;
    for(unsigned int i = 0; i<embeddedPoints.size(); i++)
    {
        for(unsigned int j = i+1; j<embeddedPoints.size(); j++)
        {
            distances[idDistance] = distancesFromEbeddedPoints(embeddedPoints[i], embeddedPoints[j]);
            idDistance++;
        }
    }

    return idDistance;

}

int interiorDistancesTest(string fileName, MyMesh& modelo, vector<vcg::Point3d>& points, vector<double>& distances, bool bVerbose)
{
    if(points.size() <= 0)
        return 0;

    vector<double> eigenValues;
    vector< vector<double> > eigenVectors;

    int eigno = getEigenVectorSolution( modelo, eigenValues, eigenVectors);

    if(bVerbose)
        printf("Se han calculado %d eigenValues\n", eigno);

    /*----------------------------------------//
         Compute the interior distance
    //----------------------------------------*/
    vector< vector<double> > embeddedPoints;
    embeddedPoints.resize(points.size());

    QTime time;
    time.start();

    // Embed coordinates.
    mvc_IBD(points, embeddedPoints, eigenVectors, modelo); // Embedding over every point to compute distance.

    float elapsedTime = time.elapsed()/1000.0;

    if(true /*bVerbose*/)
    {
        FILE* foutExtra;
        foutExtra = fopen("ownCodeResults.txt","w");

        fprintf(foutExtra,"Fichero: %s \nTiempo: %3.10e \nPuntos_interiores: %d \nEigen values: %d \n", fileName.c_str(),elapsedTime, (int)points.size(), (int)eigenVectors[0].size());
        for(unsigned int i = 0; i<embeddedPoints.size(); i++)
        {
            for(unsigned int j = 0; j<embeddedPoints[i].size(); j++)
            {
                fprintf(foutExtra,"%3.10e ",embeddedPoints[i][j]);
            }
            fprintf(foutExtra,"\n");
        }

        fclose(foutExtra);
    }

    // Distance calculus with embeded coordinates.
    distances.resize(points.size()-1);
    for(unsigned int i = 1; i<embeddedPoints.size(); i++)
        distances[i-1] = interiorDistancesFromEbeddedPoints(embeddedPoints[0], embeddedPoints[i]);

    return 0;
}

int HiResEmbedding(vector< vector<double> >& embeddingMedRes, vector< vector<double> >& embeddingHiResFromMedRes, MyMesh& modeloMedRes, MyMesh& modeloHiRes)
{
    vector<vcg::Point3d> pointsHiRes;
    pointsHiRes.resize(modeloHiRes.vn);
    MyMesh::VertexIterator vi;
    int i = 0;
    for(vi = modeloHiRes.vert.begin(); vi!=modeloHiRes.vert.end(); ++vi )
    {
        pointsHiRes[i] = vcg::Point3d(vi->P());
        i++;
    }

    embeddingHiResFromMedRes.resize(pointsHiRes.size());

    // Embed hiRes modelo with medResModel
    mvc_IBD(pointsHiRes, embeddingHiResFromMedRes, embeddingMedRes, modeloMedRes); // Embedding over every point to compute distance.
}

int interiorDistancesFromSimplificationTest(string prefix, string fileNameMedResEmbedding,string fileNameHiResEmbedding,
                                            MyMesh& modeloMedRes, MyMesh& modeloHiRes,vector<vcg::Point3d>& points,
                                            vector<double>& distances, bool bVerbose)
{
    if(points.size() <= 0)
        return 0;

    vector< vector<double> > embeddingMedRes;
    vector< vector<double> > embeddingHiRes;
    vector< vector<double> > embeddingHiResFromMedRes;

    ReadEmbedding(fileNameMedResEmbedding, embeddingMedRes);
    ReadEmbedding(fileNameHiResEmbedding, embeddingHiRes);

    HiResEmbedding(embeddingMedRes, embeddingHiResFromMedRes, modeloMedRes, modeloHiRes);

    /*
    vector<vcg::Point3d> pointsHiRes;
    pointsHiRes.resize(modeloHiRes.vn);
    MyMesh::VertexIterator vi;
    int i = 0;
    for(vi = modeloHiRes.vert.begin(); vi!=modeloHiRes.vert.end(); ++vi )
    {
        pointsHiRes[i] = vcg::Point3d(vi->P());
        i++;
    }

    // Embed hiRes modelo with medResModel
    mvc_IBD(pointsHiRes, embeddingHiResFromMedRes, embeddingMedRes, modeloMedRes); // Embedding over every point to compute distance.
    */

    /*----------------------------------------//
         Compute the interior distance
    //----------------------------------------*/
    vector< vector<double> > embeddedPoints_with_HiRes;
    embeddedPoints_with_HiRes.resize(points.size());

    vector< vector<double> > embeddedPoints_with_MedRes;
    embeddedPoints_with_MedRes.resize(points.size());

    vector< vector<double> > embeddedPoints_with_HiMedRes;
    embeddedPoints_with_HiMedRes.resize(points.size());

    // Embed coordinates.
    mvc_IBD(points, embeddedPoints_with_HiRes, embeddingHiRes, modeloHiRes); // Embedding with hi model res embeding
    mvc_IBD(points, embeddedPoints_with_MedRes, embeddingMedRes, modeloMedRes); // Embedding with med model res embeding
    mvc_IBD(points, embeddedPoints_with_HiMedRes, embeddingHiResFromMedRes, modeloHiRes); // Embedding with hi model res embeding computed from med red embeding

    FILE* foutExtra;
    foutExtra = fopen((QString(prefix.c_str())+QString("_embeddedPoints_with_medRes.txt")).toStdString().c_str(),"w");
    fprintf(foutExtra,"MEDRES \n");
    for(unsigned int i = 0; i<embeddedPoints_with_MedRes.size(); i++)
    {
        for(unsigned int j = 0; j<embeddedPoints_with_MedRes[i].size(); j++)
        {
            fprintf(foutExtra,"%3.10e ",embeddedPoints_with_MedRes[i][j]);
        }
        fprintf(foutExtra,"\n");
    }
    fclose(foutExtra);

    foutExtra = fopen((QString(prefix.c_str())+QString("_embeddedPoints_with_hiRes.txt")).toStdString().c_str(),"w");
    fprintf(foutExtra,"HiRes");
    for(unsigned int i = 0; i<embeddedPoints_with_HiRes.size(); i++)
    {
        for(unsigned int j = 0; j<embeddedPoints_with_HiRes[i].size(); j++)
        {
            fprintf(foutExtra,"%3.10e ",embeddedPoints_with_HiRes[i][j]);
        }
        fprintf(foutExtra,"\n");
    }
    fclose(foutExtra);

    foutExtra = fopen((QString(prefix.c_str())+QString("_embeddedPoints_with_MedhiRes.txt")).toStdString().c_str(),"w");
    fprintf(foutExtra,"MedHiRes");
    for(unsigned int i = 0; i<embeddedPoints_with_HiMedRes.size(); i++)
    {
        for(unsigned int j = 0; j<embeddedPoints_with_HiMedRes[i].size(); j++)
        {
            fprintf(foutExtra,"%3.10e ",embeddedPoints_with_HiMedRes[i][j]);
        }
        fprintf(foutExtra,"\n");
    }
    fclose(foutExtra);




    foutExtra = fopen((QString(prefix.c_str())+QString("_distances_with_medRes.txt")).toStdString().c_str(),"w");
    fprintf(foutExtra,"MEDRES \n");
    for(unsigned int i = 0; i<embeddedPoints_with_MedRes.size(); i++)
    {
        for(unsigned int j = i+1; j<embeddedPoints_with_MedRes.size(); j++)
        {
            double distance = interiorDistancesFromEbeddedPoints(embeddedPoints_with_MedRes[0], embeddedPoints_with_MedRes[i]);
            fprintf(foutExtra,"%3.10e\n",distance);
        }
    }
    fclose(foutExtra);

    foutExtra = fopen((QString(prefix.c_str())+QString("_distances_with_hiRes.txt")).toStdString().c_str(),"w");
    fprintf(foutExtra,"HiRes");
    for(unsigned int i = 0; i<embeddedPoints_with_HiRes.size(); i++)
    {
        for(unsigned int j = i+1; j<embeddedPoints_with_HiRes.size(); j++)
        {
            double distance = interiorDistancesFromEbeddedPoints(embeddedPoints_with_HiRes[0], embeddedPoints_with_HiRes[i]);
            fprintf(foutExtra,"%3.10e\n",distance);
        }
    }
    fclose(foutExtra);

    foutExtra = fopen((QString(prefix.c_str())+QString("_distances_with_MedhiRes.txt")).toStdString().c_str(),"w");
    fprintf(foutExtra,"MedHiRes");
    for(unsigned int i = 0; i<embeddedPoints_with_HiMedRes.size(); i++)
    {
        for(unsigned int j = i+1; j<embeddedPoints_with_HiMedRes.size(); j++)
        {
            double distance = interiorDistancesFromEbeddedPoints(embeddedPoints_with_HiMedRes[0], embeddedPoints_with_HiMedRes[i]);
            fprintf(foutExtra,"%3.10e\n",distance);
        }
    }
    fclose(foutExtra);

    return 0;
}

