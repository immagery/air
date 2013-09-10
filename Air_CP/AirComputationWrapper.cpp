/*
 * Air extension for Creation Platform
 * Air Computations wrapper
 *
 * Copyright 2013-2014 Jesus Rodriguez Nieto. All rights reserved.
 */

#include <Fabric/EDK/EDK.h>
#include <stdio.h>

#include <DataStructures/Scene.h>
#include <DataStructures/Modelo.h>
#include <DataStructures/grid3D.h>
#include <DataStructures/InteriorDistancesData.h>
#include <Computation/Segmentation.h>

#include <vcg/simplex/vertex/base.h>

using namespace Fabric::EDK::KL;
// TYPES DEFINITION \\

FABRIC_EXT_KL_STRUCT( Vec3, { float x; float y; float z; });

// WEIGHTS
FABRIC_EXT_KL_CLASS(AirWeights,{
	public:

	AirWeights()
	{
		weights.resize(0);
		indexes.resize(0);
		secondWeights.resize(0);
		points = 0;
		nodes = 0;
	}

	VariableArray< VariableArray<Scalar> > weights;
	VariableArray< VariableArray<Integer> > indexes; 

	VariableArray< VariableArray<Integer> > SecIndexes; 
	VariableArray< VariableArray<Integer> > relativeSecIndexes; 
	VariableArray< VariableArray<Scalar> > secondWeights; 

	Integer points;
	Integer nodes;

});

// BONE
FABRIC_EXT_KL_CLASS(AirBone,{
	public:

	AirBone(){}

	String name;
	Integer id;
	Integer parent;

	Vec3 position;
	Vec3 rotation;
	Vec3 orientation;
	Vec3 worldPosition;

	Scalar expansion;
	Scalar smoothness;
	Scalar stiffness;

	Boolean updatePos;
	Boolean updateParams;
});

// SKELETON
FABRIC_EXT_KL_CLASS( AirSkeleton, {
public:

	AirSkeleton(){}

	Scalar globalSmoothness;
	String name;
	Integer root;
	Boolean globalUpdate;
	VariableArray<AirBone> bone;
});

// Embedding o al menos el nombre del fichero
FABRIC_EXT_KL_CLASS( AirEmbedding, {

public:

	AirEmbedding(){}
	Integer pts;
	Integer fact;
	VariableArray< VariableArray< Scalar > >  data;

});

// Modelo... o nombre del fichero OFF
FABRIC_EXT_KL_CLASS( AirModel, {

public:
	AirModel(){}

	String name;
	String OFFfileName;
	Boolean modelLoaded;
	String embeddingFileName;
	String sPath;
});

// ALL AIR DATA
FABRIC_EXT_KL_CLASS( AirScene, {

public:
	AirScene(){}
	Boolean created;
	Boolean computation;
	AirSkeleton skt;
	AirEmbedding embedding;
	AirModel model;
	AirWeights weights;

	Data escena;
});

string toString(String str)
{
	return string(str.data());
}

// Memory allocation
FABRIC_EXT_EXPORT void AirSkinning_Create(AirScene &air)
{
	scene* escena = new scene();
	air.escena = escena;

	printf("Air memory allocated\n"); fflush(0);
};

// Set skeleton data
FABRIC_EXT_EXPORT bool AirSkinning_SetSkeleton(AirScene &air, AirSkeleton &cpSkt2)
{	
	printf("SetSkeleton function\n"); fflush(0);
	scene* snc = (scene*)air.escena;

	AirSkeleton &cpSkt = air.skt;

    skeleton* skt = new skeleton();
    skt->sName = toString(cpSkt.name);

	map<int, joint*> jointRef;

	for(Integer i = 0; i< cpSkt.bone.size(); i++)
	{
		printf("Bone %d: %s\n", i, cpSkt.bone[i].name.data());fflush(0);

		joint* jt = new joint(scene::getNewId());
		jointRef[cpSkt.bone[i].id] = jt;

		jt->sName = toString(cpSkt.bone[i].name);
		jt->smoothness = cpSkt.bone[i].smoothness;
		jt->expansion = cpSkt.bone[i].expansion;

		jt->resetTransformation();

		jt->addTranslation(cpSkt.bone[i].position.x, cpSkt.bone[i].position.y, cpSkt.bone[i].position.z);
		jt->addRotation(cpSkt.bone[i].rotation.x, cpSkt.bone[i].rotation.y, cpSkt.bone[i].rotation.z);
		jt->setJointOrientation(cpSkt.bone[i].orientation.x, cpSkt.bone[i].orientation.y, cpSkt.bone[i].orientation.z);

		jt->setWorldPosition(Point3d(
			cpSkt.bone[i].worldPosition.x, 
			cpSkt.bone[i].worldPosition.y, 
			cpSkt.bone[i].worldPosition.z ));

		if(cpSkt.bone[i].id>0)
		{
			jt->setFather(jointRef[cpSkt.bone[i].parent]);
			jointRef[cpSkt.bone[i].parent]->pushChild(jt);
		}

		skt->joints.push_back(jt);
		skt->jointRef[jt->nodeId] = jt;

		snc->defIds[jt->nodeId] = i;

		if(cpSkt.bone[i].parent < 0)
			skt->root = jt;
	}

	printf("Total bones: %d", cpSkt.bone.size()); fflush(0);

	skt->update();
	snc->skeletons.push_back(skt);

	printf("Fin set skeleton function\n"); fflush(0);

	return true;
};

// Set OFF model
FABRIC_EXT_EXPORT bool AirSkinning_LoadModel(AirScene &air, String &of, String &name, String &path)
{	
	scene* snc = (scene*)air.escena;

	if(of.data() == NULL)  { printf("El fichero OFF no esta definido\n"); fflush(0); return false;}

	if(name.data() == NULL){ printf("El name no esta definido\n"); fflush(0); return false;}

	if(path.data() == NULL){ printf("El sPath no esta definido\n"); fflush(0); return false;}

	// Load OFF file
	Modelo* model = new Modelo();
	model->loadModel(string(of.data()), 
					 string(name.data()), 
					 "off", 
					 string(path.data()));
	snc->models.push_back(model);

	model->sModelPrefix = string(name.data());
	
	printf("Model loaded: %d verts && %d faces\n", model->vn, model->fn); fflush(0);
	
	return true;
};

// Set OFF model
FABRIC_EXT_EXPORT bool AirSkinning_UpdateModel(AirScene &air)
{	
	printf("UpdateModel in\n"); fflush(0);
	scene* snc = (scene*)air.escena; 

	printf("UpdateModel out\n"); fflush(0);
	return true;
};


// Set embedding
FABRIC_EXT_EXPORT bool AirSkinning_LoadEmbedding(AirScene &air, AirEmbedding &embedding)
{	
	scene* scn = (scene*)air.escena;
	Modelo* m = (Modelo*)scn->models.back();

	m->embedding.resize(embedding.data.size());
	for(int i = 0; i< m->embedding.size(); i++)
	{
		m->embedding[i].resize(embedding.data[i].size());
		for(int j = 0; j< m->embedding[i].size(); j++)
		{
			m->embedding[i][j] = embedding.data[i][j];
		}
	}

	printf("Embedding Loaded.\n"); fflush(0);
	return true;
};

void getWeightsResults(Modelo* model, grid3d &grid, scene* scn, AirWeights& weights)
{
	printf("getWeightsResults\n"); fflush(0);

	if(model->vn != weights.points)
		printf("Hay una incongruencia entre datos %d y %d.\n",model->vn, weights.points);

	//printf("getting results %d points and %d nodes.\n",weights.points, weights.nodes);
		
	int count = 0;
	float sum = 0;
	MyMesh::VertexIterator vi;
	weights.weights.resize(weights.points);
	weights.indexes.resize(weights.points);

	weights.SecIndexes.resize(weights.points);; 
	weights.relativeSecIndexes.resize(weights.points);; 
	weights.secondWeights.resize(weights.points);; 

	for(int i = 0 ; i< weights.points; i++)
	{
		weights.weights[i].resize(0);
		weights.weights[i].resize(0);
	}

	//FILE* fout = fopen("secondaryWeights.txt", "w");
	for(vi = model->vert.begin(); vi!=model->vert.end(); ++vi ) 
	{
		if(count >= weights.points)
		{
			printf("OverBlown, count %d and points %d\n",count ,weights.points); fflush(0);
			return;
		}

		// Obtener celda
		Point3d ptPos = vi->P();
		Point3i pt = grid.cellId(ptPos);
		cell3d* cell = grid.cells[pt.X()][pt.Y()][pt.Z()];

		if(cell->getType() != BOUNDARY) printf("Esta pasando algo grave\n");

		//fprintf(fout, "Influences: %d -> ", cell->data->influences.size()); fflush(fout);

		// copiar pesos
		for(unsigned int w = 0; w< cell->data->influences.size(); w++)
		{
			float auxWeight = cell->data->influences[w].weightValue;
			int idDeformer = cell->data->influences[w].label;

			if(auxWeight > 0)
			{
				//escena->defIds[idDeformer];
				weights.weights[count].push_back(auxWeight);
				weights.indexes[count].push_back(scn->defIds[idDeformer]);
				//fprintf(fout, "(%d,  %f) ", idDeformer, auxWeight); fflush(fout);
			}
		}

		//printf("secondary: %d\n", cell->data->auxInfluences.size()); fflush(0);

		//fprintf(fout, "\n"); fflush(fout);
		//fprintf(fout, "Secondary Influences: %d -> ", cell->data->auxInfluences.size()); fflush(fout);

		for(unsigned int w = 0; w< cell->data->auxInfluences.size(); w++)
		{
			float auxWeight = cell->data->auxInfluences[w].weightValue;
			int idDeformer = cell->data->auxInfluences[w].label;
			int idRelDeformer = cell->data->auxInfluences[w].relativeLabel;

			//printf("ojo idRelDeformer: %d\n",idRelDeformer);
			//printf("ojo idDeformer: %d\n",idDeformer);
			//fflush(0);

			//escena->defIds[idDeformer];
			weights.secondWeights[count].push_back(auxWeight);
			weights.SecIndexes[count].push_back(scn->defIds[idDeformer]);
			weights.relativeSecIndexes[count].push_back(scn->defIds[idRelDeformer]);

			//fprintf(fout, "(%d,%d:%f) ", idDeformer,idRelDeformer, auxWeight); fflush(fout);
		}
		//fprintf(fout, "\n\n"); fflush(fout);
		count++;
	}

	//fclose(fout);
}

FABRIC_EXT_EXPORT bool AirSkinning_processWeights(
	AirScene &air, 
	AirWeights &result, 
	Scalar globalSmoothness, 
	Scalar sceneScale,
	Scalar kValue,
	Scalar alphaValue)
{
	scene* scn = (scene*)air.escena;

	// If is needed we compute the voxelization for every model.
	for(unsigned int i = 0; i< scn->models.size(); i++)
	{
		Modelo* m = (Modelo*)scn->models[i];

		result.points = m->vn;
		result.nodes = air.skt.bone.size();

		float gridResolution = 7;

		// De momento solo actualizamos si no hay grid
		if(!m->grid)
		{
			printf("Voxelization: only one time.\n"); fflush(0);
			Voxelize(scn, m, gridResolution, sceneScale);
		}
		//else
		//{
		//	delete m->grid;
		//	m->grid = NULL;
		//	Voxelize(scn, m, gridResolution, sceneScale);
		//}
		grid3d& grid = *(m->grid);

		grid.kValue = kValue;
		grid.alphaValue = alphaValue;
		grid.smoothPropagationRatio = globalSmoothness;

		// Lanzar el cálculo
		// No pasamos datos de updating
		ComputeSkining(m, grid);

		// Recuperar el resultado.
		getWeightsResults(m, grid, scn, result);
	}

	return true;
};

FABRIC_EXT_EXPORT void AirSkinning_Delete(AirScene &air)
{
	delete (scene*)air.escena;
	air.escena = NULL;
};

IMPLEMENT_FABRIC_EDK_ENTRIES( AirSkinning );