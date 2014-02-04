#include "BulgeDeformer.h"

#include <DataStructures\AirRig.h>
#include <DataStructures\Geometry.h>
#include <DataStructures\Modelo.h>

BulgeDeformer::BulgeDeformer()
{
	enabled = false;
}

void BulgeDeformer::applyDeformation(Geometry* m, binding* b, AirRig* rig)
{
	for(int bgIdx = 0; bgIdx < groups.size(); bgIdx++)
	{
		if(groups[bgIdx]->deformerIds.size() <= 1) continue;

		int currentDGIdx = groups[bgIdx]->deformerIds[0];
		int currentDGFatherIdx = groups[bgIdx]->deformerIds[1];

		DefGroup* dg = rig->defRig.defGroupsRef[currentDGIdx];
		DefGroup* dgParent = rig->defRig.defGroupsRef[currentDGFatherIdx];

		if(dg == NULL || dgParent == NULL) continue;

		// Suponemos que solo hay un hijo
		DefGroup* dgChild = dg->relatedGroups.back();

		if(dgChild == NULL) return;

		// Obtenemos desplazamientos.
		Vector3d v1 = dgParent->getTranslation(false) - dg->getTranslation(false);
		Vector3d v2 = dgChild->getTranslation(false) - dg->getTranslation(false);

		v1.normalize();
		v2.normalize();

		// Esto controla si estan alineados -> en ese caso no aplica ningun efecto
		if(fabs(fabs(v1.dot(v2))-1) < SMALL_NUM) return;

		Vector3d v3 = dgChild->getTranslation(false) - dgParent->getTranslation(false);
		Quaterniond q; q.setFromTwoVectors(v1,v2);
		Vector3d bisectriz = q.slerp(0.5, Quaterniond::Identity())._transformVector(v1);
		bisectriz.normalize();

		// La IK debería haber colocado bien los joints... aqui partimos de que esta todo bien alineado.
		// Obtenemos los planos centrados en dg
		Vector3d nT = v1.cross(v2).normalized();
		Vector3d nC = nT.cross(bisectriz).normalized();
		Vector3d nB = nC.cross(nT).normalized();

		Vector3d planeOrigin = dg->getTranslation(false);

		// Proyectar los puntos que toca
		vector<Vector3d> displacements(groups[bgIdx]->vertexGroup.size());
		vector<bool> displaced(groups[bgIdx]->vertexGroup.size());
		float acumPressureChild = 0;
		float acumPressureFather = 0;

		for(int k = 0; k < groups[bgIdx]->vertexGroup.size(); k++)
		{
			int idxPt = groups[bgIdx]->vertexGroup[k];
			// Proyectar el punto y guardarme el desplazamiento

			PointData* pd = &(b->pointData[idxPt]);
			Vector3d dir = (pd->node->position-planeOrigin);

			float a = dir.dot(nC);

			DefNode* df = rig->defRig.defNodesRef[pd->segmentId];

			if(a > 0 && df->boneId == currentDGFatherIdx)
			{
				// Esta asignado al padre y está en terreno del hijo
				displaced[k] = true;

				Vector3d I = -(a)*nC+ pd->node->position;
				displacements[k] = pd->node->position - I;

				acumPressureFather+= displacements[k].norm();
				
				// proyeccion talcu
				pd->node->position = I;
			}
			else if(a < 0 && df->boneId == currentDGIdx)
			{
				// Esta asignado al hijo y está en terreno del padre.
				displaced[k] = true;
				Vector3d I = -(a)*nC+ pd->node->position;
				displacements[k] = pd->node->position - I;

				acumPressureChild+= displacements[k].norm();

				// proyeccion talcu
				pd->node->position = I;
			}
			else displaced[k] = false;
		}

		printf("Preasure Child:&f Father:&f\n", acumPressureFather, acumPressureChild);
	}
}