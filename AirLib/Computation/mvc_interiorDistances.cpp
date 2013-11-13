//Raif Rustamov, Drew University
//evaluate 3D mean value interpolant
//inputs must be transposed!!!! remember fortran C differenc row first vs columns first
//follows the pseudocode in Ju et al.

//#include "mex.h"
#include "Computation/mvc_interiorDistances.h"
#include <utils/util.h>

#define  PI  (3.14159265)

using namespace std;
using namespace Eigen;

void mvcAllBindings(Vector3d& point, vector<double>& weights, Modelo& modelo)
{
	
    const double tresh1 = 0.0001;
    const double tresh2 = 0.001;
    const double tresh3 = 0.0001;

	// We use all the model
    int nopts = modelo.vn();
    if(nopts == 0)
    {
        printf("ERROR!: El modelo no está bien inicializado!\n");
        return;
    }   

	weights.clear();
    weights.resize(nopts,0.0); // incializamos el resultado

    vector< Vector3d> unitVectors;
    vector<double> normas;
    unitVectors.resize(nopts);
    normas.resize(nopts);

    //MyMesh::VertexIterator vertIt;
    //for(vertIt = modelo.vert.begin(); vertIt!=modelo.vert.end(); ++vertIt )
	for(int vertIt = 0; vertIt < modelo.nodes.size(); vertIt++ )
	{
		//Vector3d dirVec = vertIt->P() - point;
		Vector3d dirVec = point - modelo.nodes[vertIt]->position;
        double norm = dirVec.norm();
		int idVert = modelo.nodes[vertIt]->id;

        assert(idVert >= 0 && idVert < nopts); // Comprobaci—n de los valors de IMark
		
        if(norm < tresh1)
        {
            // El punto est‡ muy cerca de este vŽrtice, devolvemos los valores en V tal cual.
			weights[idVert] = 1.0;
            return; // Ya no tenemos que calcular nada m‡s para este punto.
        }
            
        unitVectors[idVert] = dirVec/norm;
        normas[idVert] = norm;
    }

    double totalW = 0;

    //MyMesh::FaceIterator fj;
    //for(fj = modelo.face.begin(); fj!=modelo.face.end(); ++fj )
	for(int fj = 0; fj < modelo.triangles.size(); fj++ )
	{
         Vector3d O, c, s;
         Vector3i idVerts;
        //for(int i = 0; i<3; i++) // Obtenemos los indices de los vertices de t
			//idVerts[i] = fj->V(i)->IMark();
			//idVerts[2-i] = fj->V(i)->IMark();

		idVerts[0] = modelo.triangles[fj]->verts[0]->id;
		idVerts[1] = modelo.triangles[fj]->verts[2]->id;
		idVerts[2] = modelo.triangles[fj]->verts[1]->id;

        for(int i = 0; i<3; i++)
        {
            double l = (unitVectors[idVerts[(i+1)%3]]- unitVectors[idVerts[(i+2)%3]]).norm();
            O[i] = 2*asin(l/2);
        }

        double h = (O[0]+O[1]+O[2])/2;

         Vector3d w; double w_sum = 0; // espacio para coordenadas y la suma

        if(M_PI - h < tresh2) // x esta sobre el triangulo t, usar coords bar 2D.
        {
            for(int i = 0; i<3; i++){  // Calculamos los valores de las coordenadas y la suma
                w[i] = sin(O[i])*normas[idVerts[(i+1)%3]]*normas[idVerts[(i+2)%3]];
                w_sum += w[i];
            }

			for(int i = 0; i<3; i++)
			{  // Guardamos la coordenada ponderada por la suma: hay que preservar la particion de unidad.
				weights[idVerts[i]] = w[i]/w_sum;
			}

            return; // Acabamos
        }

		double determ = det(unitVectors[idVerts[0]], unitVectors[idVerts[1]], unitVectors[idVerts[2]]);

        bool okValues = true;
        for(int i = 0; i<3 && okValues; i++)
        {
            c[i] = 2*sin(h)*sin(h-O[i])/(sin(O[(i+1)%3])*sin(O[(i+2)%3]))-1;
            s[i] = sign(determ)*sqrt(1-c[i]*c[i]);

            okValues &= (fabs(s[i]) > tresh3);
        }

        if(!okValues)
            continue;

        w[0] = (O[0]- c[1]*O[2] - c[2]*O[1])/(normas[idVerts[0]]*sin(O[1])*s[2]);
        w[1] = (O[1]- c[0]*O[2] - c[2]*O[0])/(normas[idVerts[1]]*sin(O[2])*s[0]);
        w[2] = (O[2]- c[1]*O[0] - c[0]*O[1])/(normas[idVerts[2]]*sin(O[0])*s[1]);

		for(int i = 0; i<3; i++)
		{  
			// Guardamos la coordenada ponderada por la suma: hay que preservar la particion de unidad.
			weights[idVerts[i]] += w[i];		
			totalW +=w[i];
		}
    }

	double sum2 = 0;
	for(int i = 0; i< weights.size(); i++)
    {
		sum2 += weights[i];
	}

	for(int i = 0; i< weights.size(); i++)
	{
		double auxWeight = weights[i];
		weights[i] = weights[i]/sum2;
	}

}

// Computes mvc for each binding.
void mvcSingleBinding( Vector3d& point, vector<double>& weights, binding* bd, Modelo& modelo)
{
	/*
	const double tresh1 = 0.0001;
    const double tresh2 = 0.001;
    const double tresh3 = 0.0001;

	int nopts = bd->pointData.size();
    if(nopts <= 0)
    {
        printf("ERROR!: No hay puntos para hacer la interpolacion mvc!\n");
        return;
    }    

	weights.clear();
	weights.resize(nopts, 0.0); // incializamos el resultado

    vector< Vector3d> unitVectors;
    vector<double> normas;
    unitVectors.resize(nopts);
    normas.resize(nopts);

	// Inicializacion y eliminacion de un caso base
	for(int pt = 0; pt < nopts; pt++)
	{
		// Nos saltamos los vertices que no toca.
		Vector3d vecDir =  bd->pointData[pt].position - point;
		double norm = vecDir.Norm();
        if(norm < tresh1)
        {
			// CASO BASE:
            // El punto esta muy cerca de este vertice, devolvemos 1 para el peso de ese punto.
            weights[pt] = 1.0;
            return;
        }
            
        unitVectors[pt] = vecDir/norm;
        normas[pt] = norm;
    }

    double totalW = 0;
    MyMesh::FaceIterator fj;
    for(fj = modelo.face.begin(); fj!=modelo.face.end(); ++fj )
    {
         Vector3d O, c, s;

		bool fromBinding = true;
         Vector3i idVerts;
        for(int i = 0; i<3; i++) // Obtenemos los indices de los vertices de t
		{
			fromBinding &=  (modelo.modelVertexBind[fj->V(i)->IMark()] == bd->bindId);
			idVerts[i] = modelo.modelVertexDataPoint[fj->V(i)->IMark()];
		}
		if(!fromBinding) continue;

        for(int i = 0; i<3; i++)
        {
            double l = (unitVectors[idVerts[(i+1)%3]]- unitVectors[idVerts[(i+2)%3]]).Norm();
            O[i] = 2*asin(l/2);
        }

        double h = (O[0]+O[1]+O[2])/2;

         Vector3d w; double w_sum = 0; // espacio para coordenadas y la suma

        if(M_PI - h < tresh2) // x esta sobre el triangulo t, usar coords bar 2D.
        {
            for(int i = 0; i<3; i++){  // Calculamos los valores de las coordenadas y la suma
                w[i] = sin(O[i])*normas[idVerts[(i+1)%3]]*normas[idVerts[(i+2)%3]];
                w_sum += w[i];
            }
			
			for(int i = 0; i<3; i++)
			{  // Guardamos la coordenada ponderada por la suma: hay que preservar la particion de unidad.
				weights[idVerts[i]] = w[i]/w_sum;
			}

            return; // Acabamos
        }

		double determ = det(unitVectors[idVerts[0]], unitVectors[idVerts[1]], unitVectors[idVerts[2]]);

        bool okValues = true;
        for(int i = 0; i<3 && okValues; i++)
        {
            c[i] = 2*sin(h)*sin(h-O[i])/(sin(O[(i+1)%3])*sin(O[(i+2)%3]))-1;
            s[i] = sign(determ)*sqrt(1-c[i]*c[i]);

            okValues &= (fabs(s[i]) > tresh3);
        }

        if(!okValues)
            continue;

        w[0] = (O[0]- c[1]*O[2] - c[2]*O[1])/(normas[idVerts[0]]*sin(O[1])*s[2]);
        w[1] = (O[1]- c[0]*O[2] - c[2]*O[0])/(normas[idVerts[1]]*sin(O[2])*s[0]);
        w[2] = (O[2]- c[1]*O[0] - c[0]*O[1])/(normas[idVerts[2]]*sin(O[0])*s[1]);

        //totalW += w[0]+w[1]+w[2];
		for(int i = 0; i<3; i++)
		{  // Guardamos la coordenada ponderada por la suma: hay que preservar la particion de unidad.
			//int bdVertIdx = modelo.modelVertexDataPoint[idVerts[i]];
			weights[idVerts[i]] += w[i];
			totalW +=w[i];
		}
        //for(int i = 0; i< nofuncs; i++) // Para cada valor de V obtenemos la interpolacion 2D
        //    embeddedPoint[i] += (V[idVerts[0]][i]*w[0]+V[idVerts[1]][i]*w[1]+V[idVerts[2]][i]*w[2]);
    }


    // Normalizacion de los valores
    //for(int i = 0; i< nofuncs; i++)
	for(int i = 0; i< nopts; i++)
        weights[i] = weights[i]/totalW;

	*/
}

/*
void mvcEmbedPoint( Vector3d& point, vector<double>& embeddedPoint, vector< vector<double> >& V, MyMesh& modelo)
{
    const double tresh1 = 0.0001;
    const double tresh2 = 0.001;
    const double tresh3 = 0.0001;

    int nopts = modelo.vn;

    if(V.size() == 0)
    {
        printf("ERROR!: No hay eigenvectors para hacer el calculo!\n");
        return;
    }
    
    if(nopts == 0)
    {
        printf("ERROR!: El modelo no está bien inicializado!\n");
        return;
    }    
	embeddedPoint.clear();
    int nofuncs = V[0].size(); // numero de eigen values a tratar
    embeddedPoint.resize(nofuncs); // incializamos el resultado
	for(int pt001 = 0; pt001 < nofuncs; pt001++)
		embeddedPoint[pt001] = 0;

    vector< Vector3d> unitVectors;
    vector<double> normas;
    unitVectors.resize(nopts);
    normas.resize(nopts);

    MyMesh::VertexIterator vertIt;
    for(vertIt = modelo.vert.begin(); vertIt!=modelo.vert.end(); ++vertIt )
    {
        double norm = (vertIt->P() - point).Norm();
        int idVert = vertIt->IMark();

        assert(idVert >= 0 && idVert < nopts); // Comprobaci—n de los valors de IMark

        if(norm < tresh1)
        {
            // El punto est‡ muy cerca de este vŽrtice, devolvemos los valores en V tal cual.
            for(int i = 0; i< nofuncs; i++)
                embeddedPoint[i] = V[idVert][i];

            return; // Ya no tenemos que calcular nada m‡s para este punto.
        }
            
        unitVectors[idVert] = (vertIt->P() - point)/norm;
        normas[idVert] = norm;
    }

    double totalW = 0;

    MyMesh::FaceIterator fj;
    for(fj = modelo.face.begin(); fj!=modelo.face.end(); ++fj )
    {
         Vector3d O, c, s;

         Vector3i idVerts;
        for(int i = 0; i<3; i++) // Obtenemos los indices de los vŽrtices de t
            idVerts[i] = fj->V(i)->IMark();

        for(int i = 0; i<3; i++)
        {
            double l = (unitVectors[idVerts[(i+1)%3]]- unitVectors[idVerts[(i+2)%3]]).Norm();
            O[i] = 2*asin(l/2);
        }

        double h = (O[0]+O[1]+O[2])/2;

         Vector3d w; double w_sum = 0; // espacio para coordenadas y la suma

        if(M_PI - h < tresh2) // x esta sobre el triangulo t, usar coords bar 2D.
        {
            for(int i = 0; i<3; i++){  // Calculamos los valores de las coordenadas y la suma
                w[i] = sin(O[i])*normas[idVerts[(i+1)%3]]*normas[idVerts[(i+2)%3]];
                w_sum += w[i];
            }

            for(int i = 0; i< nofuncs; i++) // Para cada valor de V obtenemos la interpolacion 2D
                embeddedPoint[i] = (w[0]*V[idVerts[0]][i]+w[1]*V[idVerts[1]][i]+w[2]*V[idVerts[2]][i])/w_sum;

            return; // Acabamos
        }

		double determ = det(unitVectors[idVerts[0]], unitVectors[idVerts[1]], unitVectors[idVerts[2]]);

        bool okValues = true;
        for(int i = 0; i<3 && okValues; i++)
        {
            c[i] = 2*sin(h)*sin(h-O[i])/(sin(O[(i+1)%3])*sin(O[(i+2)%3]))-1;
            s[i] = sign(determ)*sqrt(1-c[i]*c[i]);

            okValues &= (fabs(s[i]) > tresh3);
        }

        if(!okValues)
            continue;

        w[0] = (O[0]- c[1]*O[2] - c[2]*O[1])/(normas[idVerts[0]]*sin(O[1])*s[2]);
        w[1] = (O[1]- c[0]*O[2] - c[2]*O[0])/(normas[idVerts[1]]*sin(O[2])*s[0]);
        w[2] = (O[2]- c[1]*O[0] - c[0]*O[1])/(normas[idVerts[2]]*sin(O[0])*s[1]);

        totalW += w[0]+w[1]+w[2];

        for(int i = 0; i< nofuncs; i++) // Para cada valor de V obtenemos la interpolacion 2D
            embeddedPoint[i] += (V[idVerts[0]][i]*w[0]+V[idVerts[1]][i]*w[1]+V[idVerts[2]][i]*w[2]);
    }


    // Normalizacion de los valores
    for(int i = 0; i< nofuncs; i++)
        embeddedPoint[i] = embeddedPoint[i]/totalW;
}
*/

void mvc_weights(vector< Vector3d>& auxPoints, vector< vector<double> >& embeddedPoints, binding* bd, Modelo& m)
{
    printf("Interpolacion de %d elementos\n", auxPoints.size());fflush(0);

    clock_t init, final;
    init=clock();

    //int printData = (int)trunc(points.size()/100);
    for(unsigned int i = 0; i< auxPoints.size(); i++)
    {
        mvcSingleBinding(auxPoints[i], embeddedPoints[i], bd, m);
    }

    final=clock()-init;
    printf("Tiempo tardado con mvc %f\n", (double)final / ((double)CLOCKS_PER_SEC));fflush(0);
}

/*
void mvc_embedding(vector< Vector3d>& points, vector< vector<double> >& embeddedPoints, vector< vector<double> >& V, MyMesh &modelo)
{
    printf("Interpolacion de %d elementos\n", points.size());fflush(0);

    clock_t init, final;
    init=clock();

    //int printData = (int)trunc(points.size()/100);
    for(unsigned int i = 0; i< points.size(); i++)
    {
        mvcEmbedPoint(points[i], embeddedPoints[i], V, modelo);
    }

    final=clock()-init;
    printf("Tiempo tardado con mvc %f\n", (double)final / ((double)CLOCKS_PER_SEC));fflush(0);

}
*/