#include "InteriorDistancesData.h"

#include <iostream>
#include <fstream>

void WriteEmbedding(std::string fileName, vector< vector<double> >& V, bool ascii)
{
    string asciiFile = fileName.append(".txt");
    string binFile = fileName.append(".bin");

    FILE* foutascii = NULL;

    if(ascii)
        foutascii = fopen(asciiFile.c_str(), "w");

    ofstream foutbin;

    if(!ascii)
    {
        foutbin.open(binFile.c_str(), ios::out | ios::app | ios::binary);

        if(!foutbin.is_open())
        {
            printf("No he podido abrir el fichero binario.\n"); fflush(0); return;
        }
    }

    if(V.size() > 0)
    {
        int size = V.size();
        int funcSize = V[0].size();

        // Binary File size
        if(!ascii)
        {
            foutbin << size;
            foutbin << funcSize;
        }
        else
        {
            fprintf(foutascii,"%d %d\n", size, funcSize);
            fflush(foutascii);
        }

        for(int i = 0; i< size; i++)
        {

            for(int j = 0; j< funcSize; j++)
            {
                if(ascii)
                    fprintf(foutascii,"%3.10e ", V[i][j]);
                else
                    foutbin << V[i][j];
            }
            if(ascii)
                fprintf(foutascii,"\n");
        }
    }

    if(ascii)
        fclose(foutascii);
    else
        foutbin.close();
}

void ReadEmbedding(std::string fileName, vector< vector<double> >& V, bool ascii)
{
	//  Limpiamos los datos de entrada.
	for(int i = 0; i <V.size(); i++) V[i].clear();
	V.clear();

    FILE* fout = NULL;

    if(ascii)
        fout = fopen(fileName.c_str(), "r");

    if(!fout)
	{
		printf("No hay creado ningun embedding, se tendra que calcular...\n"); 
		fflush(0);
		return;
	}

    int num1 = 0, num2 = 0;
    char str[1000];

    fscanf(fout, "%s", str); // nombre fichero
    fscanf(fout, "%d", &num1);
    fscanf(fout, "%d", &num2);

    V.resize(num1);

    if(V.size() > 0)
    {
        for(int i = 0; i< num1; i++)
        {
            V[i].resize(num2);
            for(int j = 0; j< num2; j++)
            {
                float dummy = 0;
                fscanf(fout,"%e", &dummy);
                V[i][j] = dummy;
            }
        }
    }

    fclose(fout);
}

double BiharmonicDistanceP2P_sorted(vector<double>& weights, vector<int>& weightsSort, int pointIdx, binding* bd, float ext, float precomputedDistance, double threshold)
{
    assert(ext != 0);

	symMatrix& A = bd->BihDistances;
    
	double qAp = 0;
	for(int i = 0; i< weightsSort.size(); i++)
	{
		if(weights[weightsSort[i]] < threshold)
			break;

		qAp += weights[weightsSort[i]]*A.get(pointIdx,weightsSort[i]);
	}

	double distance = precomputedDistance - (2*qAp);
    return distance/ext;
}

double BiharmonicDistanceP2P_block(vector<double>& weights, int pointIdx, binding* bd, float ext, float precomputedDistance, int iniIdx, int finIdx)
{
    assert(ext != 0);

	symMatrix& A = bd->BihDistances;
    
	double qAp = 0;
	int MatrixRow = 0;
	for(int i = iniIdx; i<= finIdx; i++)
	{
		qAp += weights[i]*A.get(pointIdx,MatrixRow);
		MatrixRow++;
	}

	double distance = precomputedDistance - (2*qAp);
    return distance/ext;
}

double BiharmonicDistanceP2P(vector<double>& weights, int pointIdx, binding* bd, float ext, float precomputedDistance)
{
    assert(ext != 0);

	symMatrix& A = bd->BihDistances;
    
	double qAp = 0;
	for(int i = 0; i< A.size; i++)
		qAp += weights[i]*A.get(pointIdx,i);

	double distance = precomputedDistance - (2*qAp);
    return distance/ext;
}

double BiharmonicDistanceP2P(vector<double>& weights, int pointIdx, binding* bd, DefNode& node)
{
	float ext = node.expansion;
    assert(ext != 0);

	symMatrix& A = bd->BihDistances;
    
	double qAp = 0;
	for(int i = 0; i< A.size; i++)
		qAp += -1*weights[i]*A.get(pointIdx,i);

	double distance = node.precomputedDistances - (2*qAp);
    return distance/ext;
}

double distancesFromEbeddedPointsExtended(vector<double>& source, vector<double>& target, float ext)
{
    double distance = 0;

    for(unsigned int j = 0; j<source.size(); j++)
    {
        //printf("(%3.10e,%3.10e): %3.10e\n", source[j], target[j], pow(target[j]-source[j],2.0)); fflush(0);
        distance += pow(target[j]-source[j],2);
    }

    if(distance < 0)
        printf("Raiz negativa!! (interiorDistancesFromEbeddedPoints)\n");

    assert(ext != 0);

    return sqrt(distance)/ext;
}

double distancesFromEbeddedPoints(vector<double>& source, vector<double>& target)
{
    double distance = 0;

    for(unsigned int j = 0; j<source.size(); j++)
    {
        //printf("(%3.10e,%3.10e): %3.10e\n", source[j], target[j], pow(target[j]-source[j],2.0)); fflush(0);
        distance += pow(target[j]-source[j],2);
    }

    if(distance < 0)
        printf("Raiz negativa!! (interiorDistancesFromEbeddedPoints)\n");

    return sqrt(distance);
}
