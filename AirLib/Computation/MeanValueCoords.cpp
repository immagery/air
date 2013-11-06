#include "MeanValueCoords.h"
//#include <QDir> // TODO -> quitar!!!
#include <iostream>
#include <fstream>

#include <Utils/util.h>

using namespace std;

// Guardamos en binario el grid entero, para no tener que recalcular cada vez
void LoadMVCoordsFromFile(std::vector< std::vector<float> > &PerVertMVC, string sFileName)
{
    ifstream myfile;
    myfile.open (sFileName.c_str(), ios::in |ios::binary);
    if (myfile.is_open())
    {
        int nv = 0, nv2 = 0;
        //int nf = 0, nf2 = 0;

        // Tama–os de las estructuras de datos
        myfile.read( (char*) &nv,  sizeof(int) );
        myfile.read( (char*) &nv2, sizeof(int) );

        PerVertMVC.resize(nv);

        assert(nv2 > 0 && nv > 0);

        // Valores de los datos
        for(int i = 0; i< nv; i++)
        {
            PerVertMVC[i].resize(nv2);
            myfile.read( (char*) &PerVertMVC[i][0], sizeof(float)*nv2);
        }

    }
    myfile.close();

}

// Cargamos de disco el grid entero.
void SaveMVCoordsToFile(std::vector< std::vector<float> > &PerVertMVC, string sFileName)
{
    ofstream myfile;
    myfile.open(sFileName.c_str(), ios::binary);
    if (myfile.is_open())
    {
        /* DATOS A GUARDAR
          >> tama–o de cada vector
          >> valores de cada vector en serie
        */

        int nv = (int)PerVertMVC.size();
        int nv2 = -1;
        if(nv > 0) nv2 = PerVertMVC[0].size();

        assert(nv2 > 0 && nv > 0);

        // Tama–os de las estructuras de datos
        myfile.write((const char*) &nv, sizeof(int));
        myfile.write((const char*) &nv2, sizeof(int));

        // Valores de los datos
        for(int i = 0; i< nv; i++)
            myfile.write((const char*) &PerVertMVC[i][0], sizeof(float)*nv2);

    }
    myfile.close();
}

/*
void CalculateMeanValueCoords(MyMesh& mesh, MyMesh& cage, std::vector< std::vector<float> > &PerVertMVC, string sFileName)
{
    //FILE* fout;
    //fout = fopen(QString(QDir::currentPath()+"/log.txt").toStdString().c_str(), "w");

    double error = 0.00001;

    int cageVertNumber = cage.vn;
    int meshVertNumber = mesh.vn;

    // Inicializacion de la estructura para recoger las coordenadas.
    PerVertMVC.resize(meshVertNumber);
    for(int i = 0; i< meshVertNumber; i++)
        PerVertMVC[i].resize(cageVertNumber, 0);

    // Obtenemos los handlers para modificar las coordenadas
     Vector3d zero; zero.SetZero(); // Valor cero por defecto para pasar por parÃ¡metro.

    MyMesh::VertexIterator pt;

    //fprintf(fout,"Inicio del calculo de las coordenadas\n\nVertices malla: %d\n verts caja: %d, triangulos caja: %d\n\n", meshVertNumber, cageVertNumber, cageFaceNumber); fflush(fout);
    for(pt = mesh.vert.begin(); pt!=mesh.vert.end(); ++pt ){

        vector< Vector3d> unitVectors;
        vector<double> normas;
        unitVectors.resize(cage.vn);
        normas.resize(cage.vn);

        MyMesh::VertexIterator cagePt;
        bool nextVert = false;
        for(cagePt = cage.vert.begin(); cagePt!=cage.vert.end(); ++cagePt )
        {
            double norm = (cagePt->P() - pt->P()).Norm();
            normas[cagePt->IMark()] = norm;
            if(norm < error)
            {
                PerVertMVC[pt->IMark()][cagePt->IMark()] = 1.0; // Queda por ver c—mo es el c‡lculo de coord en este caso.
                nextVert = true;
                break;
            }
            else
            {
                unitVectors[cagePt->IMark()] = (cagePt->P() - pt->P())/norm;
            }
        }

        if(nextVert) continue; // Ya no tenemos que calcular m‡s coordenas para este punto.

        //double totalF = 0, totalW = 0;

         //int vertNum = pt->IMark();
         MyMesh::FaceIterator fj;
         for(fj = cage.face.begin(); fj!=cage.face.end(); ++fj ){

              Vector3d O, c, s;

             for(int i = 0; i<3; i++)
             {
                 double l = (unitVectors[fj->V((i+1)%3)->IMark()]- unitVectors[fj->V((i+3-1)%3)->IMark()]).Norm();
                 O[i] = 2*asin(l/2);
             }

             double h = 0;
             for(int i = 0; i<3; i++) h += O[i];
             h /= 2;

             if(M_PI - h < error)
             {
                  Vector3d w;
                 // x esta sobre t, usar coords bar 2D.
                 for(int i = 0; i<3; i++)
                 {
                     // Calculamos los valores de las coordenadas
                     PerVertMVC[pt->IMark()][fj->V(i)->IMark()] = sin(O[i])*normas[fj->V((i+3-1)%3)->IMark()]*normas[fj->V((i+1)%3)->IMark()];
                     nextVert = true;
                     break;
                 }

                 if(nextVert) break;

             }

             if(nextVert) continue; // Ya no tenemos que calcular m‡s coordenas para este punto.

             double determ = det(unitVectors[fj->V(0)->IMark()], unitVectors[fj->V(1)->IMark()], unitVectors[fj->V(2)->IMark()]);
             for(int i = 0; i<3; i++)
             {
                 c[i] = (2*sin(h)*sin(h-O[i]))/(sin(O[(i+3-1)%3])*sin(O[(i+1)%3]))-1;
                 s[i] = sign(determ)*sqrt(1-c[i]*c[i]);
                 if(fabs(s[i]) <= error)
                 {
                     nextVert = true;
                     break;
                 }
             }

             if(nextVert) continue;

             for(int i = 0; i<3; i++)
             {
                 PerVertMVC[pt->IMark()][fj->V(i)->IMark()] += (O[i]-c[(i+1)%3]*O[(i+3-1)%3] - c[(i+3-1)%3]*O[(i+1)%3])/(normas[fj->V(i)->IMark()]*sin(O[(i+1)%3])*s[(i+3-1)%3]);
             }

         }
    }

    // Normalizaci—n de las coordenadas
    for(unsigned int i = 0; i< PerVertMVC.size(); i++)
    {
        double sum = 0;
        for(unsigned int j = 0; j< PerVertMVC[i].size(); j++)
        {
            sum += PerVertMVC[i][j];
        }

        for(unsigned int j = 0; j< PerVertMVC[i].size(); j++)
        {
            PerVertMVC[i][j] = PerVertMVC[i][j] / sum;
        }
    }

    //printf("Valores max y min %f, %f\n", minV, maxV);
    //fclose(fout);

    if(!sFileName.empty())
        SaveMVCoordsToFile(PerVertMVC, sFileName.c_str());
}
*/
/*
void deformMeshWithMVC(MyMesh& mesh, MyMesh& cage, MyMesh& newMesh, MyMesh& newCage, std::vector< std::vector<float> >& PerVertMVC)
{
    bool coordDebug = false;
    double minV = 99999;
    double maxV = -99999;
    float minC = 9999;
    float maxC = -99999;
    MyMesh::VertexIterator pt;
    MyMesh::VertexIterator Oldpt;
    Oldpt = mesh.vert.begin();
    // Este es el c‡lculo simple, pero quiz‡s deber’a hacerlo segœn desplazamientos sobre el punto original.
    for(pt = newMesh.vert.begin(); pt!=newMesh.vert.end(); ++pt )
    {

         Vector3d ptAux(0,0,0);

        double coordVertSum = 0;
        MyMesh::VertexIterator newcagePt;
        MyMesh::VertexIterator cagePt;
        cagePt = cage.vert.begin();
        for(newcagePt = newCage.vert.begin(); newcagePt!=newCage.vert.end();)
        {
            ptAux += (newcagePt->P()-cagePt->P())*PerVertMVC[pt->IMark()][newcagePt->IMark()]; // Acumulamos los desplazamientos

            cagePt++; newcagePt++;

            if(coordDebug)coordVertSum += PerVertMVC[pt->IMark()][newcagePt->IMark()];
            if(coordDebug) minC = min(minC, PerVertMVC[pt->IMark()][newcagePt->IMark()]);
            if(coordDebug) maxC = max(maxC, PerVertMVC[pt->IMark()][newcagePt->IMark()]);
        }

            if(coordDebug) minV = min(minV, coordVertSum);
            if(coordDebug) maxV = max(maxV, coordVertSum);

        pt->P() = Oldpt->P() + ptAux;
        Oldpt++;
    }

        if(coordDebug) printf("Suma para de coord esta entre el %f y el %f\n", minV, maxV);
        if(coordDebug) printf("Coord esta entre el %f y el %f\n", minC, maxC);
        fflush(0);
}
*/

