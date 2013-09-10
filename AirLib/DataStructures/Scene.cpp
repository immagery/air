#include "Scene.h"
#include "..\render\gridRender.h"

unsigned int scene::sceneIds = FIRST_NODE_ID;


void scene::setSceneScale( float sceneScale)
{
	if(DEBUG) printf("setSceneScale %f\n", sceneScale); fflush(0);

	for(unsigned int i = 0; i< visualizers.size(); i++)
	{
		((gridRenderer*)visualizers[i])->grid->worldScale = sceneScale;
	}
}

void scene::setGlobalSmoothness(float globalSmooth)
{
	if(DEBUG) printf("setGlobalSmoothness %f\n", globalSmooth); fflush(0);

	for(unsigned int i = 0; i< visualizers.size(); i++)
	{
		((gridRenderer*)visualizers[i])->grid->smoothPropagationRatio = globalSmooth;
	}
}

