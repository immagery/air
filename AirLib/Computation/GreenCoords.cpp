#include "GreenCoords.h"
//#include <QDir>
#include <iostream>
#include <fstream>

#include <utils\util.h>
#include <cmath>

using namespace std;

// devuelve el signo del valor de entrada.
/*double valueSign(double v)
{
	if(v >= 0)
		return 1;
	else
		return -1;
}*/

double GCTriInt(vcg::Point3d p, vcg::Point3d v1, vcg::Point3d v2, vcg::Point3d pt)
{
    double alfa, beta, lambda, c;
    double angles[2], I[2];

    double V1_p_norm = (p-v1).Norm();

    alfa = acos(((v2-v1)*(p-v1))/((v2-v1).Norm()*V1_p_norm));
    beta = acos(((v1-p)*(v2-p))/(V1_p_norm*((v2-p).Norm())));

    lambda = V1_p_norm * V1_p_norm * sin(alfa) * sin(alfa);
    c = pow((p-pt).Norm(),2);

    angles[0] = M_PI - alfa;
    angles[1] = M_PI - alfa - beta;

    double cRoot = sqrt(c);
    for(int i = 0; i< 2; i++)
    {
        double S = sin(angles[i]);
        double C = cos(angles[i]);

        double atanInterior = (cRoot*C)/sqrt(lambda+S*S*c);
        double firstPart = 2.0*cRoot*atan(atanInterior);

        double value1 = (2*sqrt(lambda)*S*S)/((1-C)*(1-C));
        double value2 = (2*c*C)/(c*(1+C)+lambda+sqrt(lambda*lambda+lambda*c*S*S));

        double secondPart = sqrt(lambda)*log(value1*(1-value2));

        I[i] = (-sign(S)/2.0)*(firstPart+secondPart);
    }

    return (-1/(4*M_PI))*fabs(I[0]-I[1]-cRoot*beta);
}


// Guardamos en binario el grid entero, para no tener que recalcular cada vez
void LoadGCCoordsFromFile(std::vector< std::vector<float> > &PerVertGC, std::vector< std::vector<float> > &PerFaceGC, string sFileName)
{
    ifstream myfile;
    myfile.open (sFileName.c_str(), ios::in |ios::binary);
    if (myfile.is_open())
    {
        int nv = 0, nv2 = 0, nf = 0, nf2 = 0;

        // Tama�os de las estructuras de datos
        myfile.read( (char*) &nv,  sizeof(int) );
        myfile.read( (char*) &nv2, sizeof(int) );
        myfile.read( (char*) &nf,  sizeof(int) );
        myfile.read( (char*) &nf2, sizeof(int) );

        PerVertGC.resize(nv);
        PerFaceGC.resize(nf);

        assert(nv2 > 0 && nf2 > 0);

        // Valores de los datos
        for(int i = 0; i< nv; i++)
        {
            PerVertGC[i].resize(nv2);
            myfile.read( (char*) &PerVertGC[i][0], sizeof(float)*nv2);
        }

        for(int i = 0; i< nf; i++)
        {
            PerFaceGC[i].resize(nf2);
            myfile.read( (char*) &PerFaceGC[i][0], sizeof(float)*nf2);
        }

    }
    myfile.close();

}

// Cargamos de disco el grid entero.
void SaveGCCoordsToFile(std::vector< std::vector<float> > &PerVertGC, std::vector< std::vector<float> > &PerFaceGC, string sFileName)
{
    ofstream myfile;
    myfile.open(sFileName.c_str(), ios::binary);
    if (myfile.is_open())
    {
        /* DATOS A GUARDAR
          >> tama�o de cada vector
          >> valores de cada vector en serie
        */

        int nv = (int)PerVertGC.size();
        int nf = (int)PerFaceGC.size();

        int nv2 = -1, nf2 = -1;

        if(nv > 0) nv2 = PerVertGC[0].size();
        if(nf > 0) nf2 = PerFaceGC[0].size();

        assert(nv2 > 0 && nf2 > 0);

        // Tama�os de las estructuras de datos
        myfile.write((const char*) &nv, sizeof(int));
        myfile.write((const char*) &nv2, sizeof(int));
        myfile.write((const char*) &nf, sizeof(int));
        myfile.write((const char*) &nf2, sizeof(int));

        // Valores de los datos
        for(int i = 0; i< nv; i++)
            myfile.write((const char*) &PerVertGC[i][0], sizeof(float)*nv2);

        for(int i = 0; i< nf; i++)
            myfile.write((const char*) &PerFaceGC[i][0], sizeof(float)*nf2);

    }
    myfile.close();
}


void gpCalculateGreenCoordinates(MyMesh& mesh, MyMesh& cage, std::vector< std::vector<float> > &PerVertGC, std::vector< std::vector<float> > &PerFaceGC, string sSavingFile)
{
	//TODEBUG: quitar referencias a QT
	/*
    FILE* fout;
    fout = fopen(QString(QDir::currentPath()+"/log.txt").toStdString().c_str(), "w");

    double error = 0.000;

    int cageVertNumber = cage.vn;
    int cageFaceNumber = cage.fn;

    int meshVertNumber = mesh.vn;

    // Inicializacion de la estructura para recoger las coordenadas.
    PerVertGC.resize(meshVertNumber);
    PerFaceGC.resize(meshVertNumber);
    for(int i = 0; i< meshVertNumber; i++)
    {
        PerVertGC[i].resize(cageVertNumber, 0);
        PerFaceGC[i].resize(cageFaceNumber, 0);
    }

    // Obtenemos los handlers para modificar las coordenadas
    vcg::Point3d zero; zero.SetZero(); // Valor cero por defecto para pasar por parámetro.

    MyMesh::VertexIterator pt;

   // double minV = 9999;
   // double maxV = -9999;

    fprintf(fout,"Inicio del cálculo de las coordenadas\n\nVertices maya: %d\n verts caja: %d, triangulos caja: %d\n\n", meshVertNumber, cageVertNumber, cageFaceNumber); fflush(fout);
    for(pt = mesh.vert.begin(); pt!=mesh.vert.end(); ++pt ){

        int vertNum = pt->IMark();
         MyMesh::FaceIterator fj;
         for(fj = cage.face.begin(); fj!=cage.face.end(); ++fj ){

             vcg::Point3d vj[3], q, N[3], w; w.SetZero();
             double I[3], II[3], s[3];

             vcg::Point3d nj(fj->N()); // normal de la cara
             nj = nj / nj.Norm();

             // Proyección del punto de la maya en el triángulo
             for(int l = 0; l<3; l++)  vj[l] = vcg::Point3d(fj->V(l)->P() - pt->P());

             vcg::Point3d p(nj*(vj[0]*nj));

             for(int i = 0; i<3; i++)
             {
                 int imas = (i+1)%3;
                 s[i] = sign(((vj[i] - p)^(vj[imas] - p))*nj);
                 I[i] = GCTriInt(p, vj[i], vj[imas], zero);
                 II[i] = GCTriInt(zero, vj[imas], vj[i], zero);
                 q = vj[imas]^vj[i];
                 N[i] = q/q.Norm();
             }

             double Iscalar = 0;
             for(int i = 0; i<3; i++)
                 Iscalar += s[i]*I[i];

             Iscalar = -fabs(Iscalar);
             int idFace = fj->IMark();

             PerFaceGC[vertNum][idFace] = -Iscalar;

             w = nj*Iscalar;
             for(int i = 0; i<3; i++)
                 w += N[i]*II[i];

            // minV = min(w.Norm(), minV);
            // maxV = max(w.Norm(), maxV);

             if(w.Norm() > error)
             {
                 for(int l = 0; l<3; l++)
                 {
                     int lmas = (l+1)%3;
                     int idVert = fj->V(l)->IMark();

                     double aportacion =  (N[lmas]*w)/(N[lmas]*vj[l]);
                     PerVertGC[vertNum][idVert] += aportacion;
                 }
             }
             else
             {
                 // Podemos analizar...
             }
         }
    }

    //printf("Valores max y min %f, %f\n", minV, maxV);
    fclose(fout);

    if(!sSavingFile.empty())
        SaveGCCoordsToFile(PerVertGC, PerFaceGC, sSavingFile.c_str());

	*/
}

void deformMeshWithGC(MyMesh& mesh, MyMesh& cage, MyMesh& newMesh, MyMesh& newCage, std::vector< std::vector<float> >& PerVertGC, std::vector< std::vector<float> >& PerFaceGC)
{
    MyMesh::VertexIterator pt;

    std::vector<double> scaleFactors;
    scaleFactors.resize(cage.fn, 1.0);

	double rootEight = sqrt(8.0);

    MyMesh::FaceIterator cageFace;
    cageFace = cage.face.begin();
    for(MyMesh::FaceIterator newCageFace = newCage.face.begin(); newCageFace!=newCage.face.end(); ++newCageFace )
    {
        vcg::Point3d u1 = cageFace->V(1)->P() - cageFace->V(0)->P();
        vcg::Point3d v1 = cageFace->V(2)->P() - cageFace->V(0)->P();

        vcg::Point3d u2 = newCageFace->V(1)->P() - newCageFace->V(0)->P();
        vcg::Point3d v2 = newCageFace->V(2)->P() - newCageFace->V(0)->P();

        vcg::Point3d Vu = (cageFace->V(2)->P() - cageFace->V(0)->P());
        Vu = Vu / Vu.Norm();
        Vu = Vu*(Vu*u1);

        double area = ((cageFace->V(0)->P()+Vu-cageFace->V(1)->P()).Norm()*(cageFace->V(2)->P() - cageFace->V(0)->P()).Norm()) /2;

        double factor = sqrt(pow(u2.Norm(),2)*pow(v1.Norm(),2) - 2*(u2*v2)*(u1*v1) + pow(v2.Norm(),2)*pow(u1.Norm(),2))/(rootEight*area);
        scaleFactors[newCageFace->IMark()] = factor;

        ++cageFace;
    }

    //double minV = 100;
    //double maxV = -100;

    for(pt = newMesh.vert.begin(); pt!=newMesh.vert.end(); ++pt )
    {
        vcg::Point3d ptAux(0,0,0);

        //double coordVertSum = 0;
        MyMesh::VertexIterator cagePt;
        for(cagePt = newCage.vert.begin(); cagePt!=newCage.vert.end(); cagePt++ )
        {
            ptAux += cagePt->P()*PerVertGC[pt->IMark()][cagePt->IMark()];
            //coordVertSum += PerVertGC[pt->IMark()][cagePt->IMark()];
        }

        //minV = min(minV, coordVertSum);
        //maxV = max(maxV, coordVertSum);

        MyMesh::FaceIterator newCageFace2;
        for(newCageFace2 = newCage.face.begin(); newCageFace2!=newCage.face.end(); newCageFace2++ )
        {
            ptAux += newCageFace2->N()*PerFaceGC[pt->IMark()][newCageFace2->IMark()]*scaleFactors[newCageFace2->IMark()];
            cageFace++;
        }

        pt->P() = ptAux;
    }

    //printf("Suma para de coord esta entre el %f y el %f\n", minV, maxV); fflush(0);
}


