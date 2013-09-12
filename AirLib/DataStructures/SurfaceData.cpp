#include "SurfaceData.h"


bool BuildSurfaceFromOFFFile(SurfaceGraph& graph, string sFileName)
{
	FILE* fin;
    fin = fopen(sFileName.c_str(), "r");

	if(!fin) return false;

	char str[1000];
    fscanf(fin, "%s", str); // Format flag
	string formatStr(str);
	if(formatStr.compare("OFF") != 0) return false;

	int num1, num2, num3;
    fscanf(fin, "%d", &num1);
    fscanf(fin, "%d", &num2);
	fscanf(fin, "%d", &num3);

	//Leemos los vertices
	graph.nodes.resize(num1);
	float value1, value2, value3;
	for(int i = 0; i< num1; i++)
	{
		fscanf(fin, "%f", &value1);
		fscanf(fin, "%f", &value2);
		fscanf(fin, "%f", &value3);
		graph.nodes[i] = new GraphNode(i);
		graph.nodes[i]->position = Point3d(value1,value2,value3);
	}

	// Leemos las caras.
	graph.triangles.resize(num2);
	int polygonCount = 0;
	int vertIdx = 0;
	for(int i = 0; i< num2; i++)
	{
		fscanf(fin, "%i", &polygonCount);
		graph.triangles[i] = new GraphNodePolygon(i);
		graph.triangles[i]->verts.resize(polygonCount);

		vector<int> indexes;
		for(int j = 0; j< polygonCount; j++)
		{
			fscanf(fin, "%i", &vertIdx);
			graph.triangles[i]->verts[j] = graph.nodes[vertIdx];
			indexes.push_back(vertIdx);
		}

		for(int j = 0; j< polygonCount; j++)
		{
			graph.nodes[indexes[j]]->connections.push_back(graph.nodes[indexes[(j+1)%polygonCount]]);
			graph.nodes[indexes[(j+1)%polygonCount]]->connections.push_back(graph.nodes[indexes[j]]);
		}
	}

	//// De momento no leemos mas que vertices y caras.
	//for(int i = 0; i< num3; i++)
	//{
	//}

	 return true;
}

PointData::PointData()
{
    influences.clear();
    auxInfluences.clear();

    embedding.clear();
    vertexContainer = false;
    itPass =-1;

    confidenceLevel = 0;

    ownerLabel = -1;
    ownerWeight = 0;

    segmentId = -1;

    domain = 0.0;
    domainId = -1;

    component = -1;
    assigned = false;
    validated = false;

	//modelVert = -1;

    color = Point3f(0,0,0);

	isBorder = false;
}

PointData::PointData(const PointData& pd)
{
	influences.resize(pd.influences.size());
	for(int i = 0; i< pd.influences.size(); i++)
		influences[i] = pd.influences[i]; 

	auxInfluences.resize(pd.auxInfluences.size());
	for(int i = 0; i< pd.auxInfluences.size(); i++)
		auxInfluences[i] = pd.auxInfluences[i]; 

	embedding.resize(pd.embedding.size());
	for(int i = 0; i< pd.embedding.size(); i++)
		embedding[i] = pd.embedding[i]; 

	vertexContainer = pd.vertexContainer;
    itPass =-1;

    confidenceLevel = pd.confidenceLevel;

    ownerLabel = pd.ownerLabel;
    ownerWeight = pd.ownerWeight;

    segmentId = pd.segmentId;

    domain = pd.domain;
    domainId = pd.domainId;

    component = pd.component;
    assigned = pd.assigned;
    validated = pd.validated;

	//modelVert = pd.modelVert;

    color = pd.color;
	node = pd.node;

	isBorder = pd.isBorder;

}

PointData::PointData(int weightsSize)
{
    if(weightsSize > 0)
        influences.resize(weightsSize);
    else
        influences.clear();

    vertexContainer = false;

    auxInfluences.clear();
    itPass =-1;
    ownerLabel = -1;
    confidenceLevel = 0;

    component = -1;
    assigned = false;
    validated = false;

    color = Point3f(0,0,0);

	isBorder = false;
}

// Clear no del todo, no quitamos el labeling de la voxelizacion
void PointData::clear()
{
    auxInfluences.clear();
    influences.clear();

    component = -1;
    assigned = false;

    confidenceLevel = 0;
    itPass =-1;

    // TO DEBUG
    //vertexContainer;
    //Point3f color;
    color = Point3f(0,0,0);

    // No tocaria
    //vector<double> embedding;

    ownerLabel = -1;
    ownerWeight = 0;
    confidenceLevel = 0;

    segmentId = -1;

    domain = 0.0;
    domainId = -1;

    component = -1;
    assigned = false;
    validated = false;
}

void PointData::clearAll()
{
    clear();
    embedding.clear();
}

void PointData::SaveToFile(ofstream& myfile)
{
    // bool vertexContainer
    myfile.write((const char*) &vertexContainer, sizeof(bool));

    // vector<weight> influences
    vector<int> labels;
    vector<float> weights;

    labels.resize(influences.size());
    weights.resize(influences.size());
    for(int i = 0; i< influences.size(); i++)
    {
        labels[i] = influences[i].label;
        weights[i] = influences[i].weightValue;
    }

    // vector<float> weights
    int size = weights.size();
    myfile.write((const char*) &size, sizeof(unsigned int));
    if(size > 0)
        myfile.write((const char*) &weights[0], sizeof(float)*weights.size());

    // vector<int> labels
    size = labels.size();
    myfile.write((const char*) &size, sizeof(unsigned int));
    if(size > 0)
        myfile.write((const char*) &labels[0], sizeof(int)*labels.size());

    // vector<double> embedding
    size = embedding.size();
    myfile.write((const char*) &size, sizeof(unsigned int));
    if(size > 0)
        myfile.write((const char*) &embedding[0], sizeof(double)*embedding.size());

    //int segmentId;
    myfile.write((const char*) &segmentId, sizeof(int));

}

void PointData::LoadFromFile(ifstream& myfile)
{
    myfile.read( (char*) &vertexContainer, sizeof(bool) );

    int size;
    vector<int> al;
    vector<float> aw;

    // Weights
    myfile.read( (char*) &size, sizeof(int) );
    if(size > 0)
    {
        aw.resize(size);
        myfile.read( (char*) &aw[0], sizeof(float)*size);
    }
    else aw.clear();

    // labels
    myfile.read( (char*) &size, sizeof(int) );
    if(size > 0)
    {
        al.resize(size);
        myfile.read( (char*) &al[0], sizeof(int)*size);
    }
    else al.clear();

    if(size > 0)
    {
        influences.resize(size);
        for(int i = 0; i< influences.size(); i++)
        {
            influences[i].label = al[i];
            influences[i].weightValue = aw[i];
        }
    }
    else
        influences.clear();

    myfile.read( (char*) &size, sizeof(int) );
    if(size > 0)
    {
        embedding.resize(size);
        myfile.read( (char*) &embedding[0], sizeof(double)*size);
    }
    else embedding.clear();

    myfile.read( (char*) &segmentId, sizeof(int) );

}

void cleanZeroInfluences(binding* bd)
{
    for(int i = 0; i< bd->pointData.size();i++)
    {
        PointData* pd = &(bd->pointData[i]);

        // Eliminamos los pesos igual a 0
        pd->auxInfluences.clear();
        for(int infl = 0; infl < pd->influences.size(); infl++)
        {
            if(pd->influences[infl].weightValue > 0)
                pd->auxInfluences.push_back(pd->influences[infl]);

			else if(pd->influences[infl].weightValue < 0)
			{
				//Comentado temporalmente
				//printf("[cleanZeroInfluences] - Pesos negativos...no deberia.\n");
				fflush(0);
			}
        }
        pd->influences.clear();
        
		for(int infl = 0; infl < pd->auxInfluences.size(); infl++)
        {
            pd->influences.push_back(pd->auxInfluences[infl]);
        }
		pd->auxInfluences.clear();
    }
}

void normalizeWeightsByDomain(binding *bd)
{
    for(int i = 0; i< bd->pointData.size(); i++)
    {
        PointData* pd = &(bd->pointData[i]);

        float childGain = 0;
        for(int infl = 0; infl< pd->auxInfluences.size(); infl++)
        {
			// DEBUG info -> to comment
			float err = (1/pow(10.0,5.0));
            if(pd->auxInfluences[infl].weightValue < 0 || (pd->auxInfluences[infl].weightValue - 1.0)>err)
			{
				// Algo está pasando...
                //printf("hay algun problema en la asignacion de pesos.\n");
			}

            childGain += pd->auxInfluences[infl].weightValue;
        }

        if(childGain == 0)
        {
            pd->auxInfluences.clear();
            continue;
        }

        if(pd->domain > childGain)
        {
            //printf("\n\nEn principio aqui no entra, porque el padre tambien juega.\n\n"); fflush(0);
            if(pd->domainId >= 0)
            {
                // Obtener la influencia del padre y quitar lo que tocaria.
                int fatherId = findWeight(pd->influences, pd->domainId);
                if(fatherId >= 0)
                {
                    pd->influences[fatherId].weightValue = pd->domain - childGain;
                }
            }

            for(int infl = 0; infl< pd->auxInfluences.size(); infl++)
            {
                int l = pd->auxInfluences[infl].label;
                float w = pd->auxInfluences[infl].weightValue;
                pd->influences.push_back(weight(l,w));
            }
        }
        else
        {
            // Eliminamos el peso del padre, porque lo repartiremos.
            if(pd->domainId >= 0)
            {
                int fatherId = findWeight(pd->influences, pd->domainId);
                if(fatherId >= 0)
                {
                    if(pd->influences[fatherId].weightValue != pd->domain)
                    {
                        printf("Hay un problema de inicializacion del dominio...\n");
                        fflush(0);
                    }

                    pd->influences[fatherId].weightValue = 0;
                }
            }

            for(int infl = 0; infl< pd->auxInfluences.size(); infl++)
            {
                int l = pd->auxInfluences[infl].label;
                float w = (pd->auxInfluences[infl].weightValue/childGain)*pd->domain;

                float a = w;
                if(a != w)
                {
                    int problema = 1;
                    printf("Tenemos un problema curioso.\n");
                }

                pd->influences.push_back(weight(l,w));
            }
        }

        pd->auxInfluences.clear();

    }

}
