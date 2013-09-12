#ifdef WIN32
#include <glew.h>
#endif

#ifdef WIN64
#include <glew.h>
#endif

#define VERBOSE false

// Qt libraries
#include <QtCore/QDir>
#include <QtWidgets/QMessageBox>
#include <QtGui/QColor>

#include "GLWidget.h"

#include <computation/GreenCoords.h>
#include <computation/HarmonicCoords.h>

#include <utils/util.h>
#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <DataStructures/Scene.h>
#include <DataStructures/Cage.h>
#include <DataStructures/Modelo.h>
#include <render/ShadingNode.h>

#include <render/gridRender.h>
#include <render/clipingPlaneRender.h>

#include "manipulatedFrameSetConstraint.h"

#include "DataStructures/InteriorDistancesData.h"
#include "Computation\BiharmonicDistances.h"
#include "Computation\mvc_interiorDistances.h"

#include "Computation/Segmentation.h"
#include "utils/ioWeights.h"

#include <Eigen/Dense>

#include <vcg/simplex/face/jumping_pos.h>

#include <iostream>
#include <fstream>

#include <QtCore/QTextStream>

#define ratioExpansion_DEF 0.7

using namespace qglviewer;
using namespace vcg;

GLWidget::GLWidget(QWidget * parent , const QGLWidget * shareWidget, Qt::WindowFlags flags)
        : QGLViewer(parent, shareWidget, flags)
{
    selectionMode_ = NONE;

    // Scene variables.
    m_ptCenter.SetZero();
    m_sceneRadius = 20000;

    m_sceneMin[0] = -m_sceneRadius;
    m_sceneMin[1] = -m_sceneRadius;
    m_sceneMin[2] = -m_sceneRadius;

    m_sceneMax[0] = m_sceneRadius;
    m_sceneMax[1] = m_sceneRadius;
    m_sceneMax[2] = m_sceneRadius;

    initScene();

    // State flags
    loadedModel = false;
    firstInit = true;
    refreshFlag = true;
    firstDrawing = true;
    showDeformedModel = false;

    // Drawing flags.
    drawCage = true;
    shadeCoordInfluence = true;
    bGCcomputed = false;
    bHComputed = false;

    //m_bHCGrid = false;

    bGCcomputed = false;
    bHComputed = false;
    bMVCComputed = false;

    influenceDrawingIdx = 0;

    //activeCoords = HARMONIC_COORDS;
    //m.sModelPath = "";

    stillCageAbled = false;
    stillCageSelected = -1;

    viewingMode = SimpleModel_Mode;
    ShadingModeFlag = SH_MODE_SMOOTH; //smooth

    // Guardamos un directorio de trabajo para ir m�s r�pido
    QFile file(QDir::currentPath()+"/WorkDir.txt");
    file.open(QFile::ReadOnly);
    if(file.exists())
    {
        QTextStream in(&file);
        sPathGlobal = in.readLine();
        QFile path(sPathGlobal);
        if(!path.exists())
        {
            printf("el directorio de trabajo no existe\n");
            sPathGlobal = QDir::currentPath();
        }
    }
    else
         sPathGlobal = QDir::currentPath();

    printf("Path de trabajo: %s\n", sPathGlobal.toStdString().c_str()); fflush(0);

    file.close();

    setTextIsEnabled(false);
    setFPSIsDisplayed(true);
    setAxisIsDrawn(false);

    colorsLoaded= false;
    colorLayers = false;
    colorLayerIdx = -1;

    escena = new scene();

	CurrentProcessJoints.clear();

	interiorPoint = Point3d(0,0,0);

    //ctxMode = CTX_SELECTION;
}

 GLWidget::~GLWidget()
 {
    //makeCurrent();
    //releaseKeyboard();
 }

 void GLWidget::init()
 {
    // A ManipulatedFrameSetConstraint will apply displacements to the selection
    /*
    setManipulatedFrame(new ManipulatedFrame());
    manipulatedFrame()->setConstraint(new ManipulatedFrameSetConstraint());
    ManipulatedFrameSetConstraint* mfsc = (ManipulatedFrameSetConstraint*)(manipulatedFrame()->constraint());
    mfsc->newCage = &m.dynCage;
    */

    // Used to display semi-transparent relection rectangle
    glBlendFunc(GL_ONE, GL_ONE);

    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    glDepthFunc(GL_LEQUAL);

    restoreStateFromFile();
    startAnimation();

    //testScene();
 }

 void GLWidget::animate()
 {
     /*
     ManipulatedFrameSetConstraint* mfsc = (ManipulatedFrameSetConstraint*)(manipulatedFrame()->constraint());
     if(!mfsc)
         return;

     if(mfsc->transformed)
         deformMesh();

     */
 }

void GLWidget::loadSelectableVertex(Cage* cage /* MyMesh& cage*/)
{
    // TODO
    /*
    MyMesh::VertexIterator vi;
    for(vi = cage.vert.begin(); vi!=cage.vert.end(); ++vi )
    {
        DrawObject* o = new DrawObject();
        o->frame.setPosition(Vec(vi->P()[0], vi->P()[1], vi->P()[2])); // Posicion
        o->id = vi->IMark(); // id
        objects_.append(o);
    }
    */
}

void GLWidget::selectElements(vector<unsigned int > lst)
{
    escena->removeSelection();
    escena->selectElements(lst);

    selMgr.selection.clear();
    for(unsigned int i = 0; i< lst.size(); i++)
    {
        for(unsigned int m = 0; m< escena->models.size(); m++)
        {
            if(((object*)escena->models[m])->nodeId == lst[i])
            {
               selMgr.selection.push_back(escena->models[m]);
            }
        }

        for(unsigned int m = 0; m< escena->skeletons.size(); m++)
        {
            skeleton* skt = ((skeleton*)escena->skeletons[m]);
            for(unsigned int j = 0; j< skt->joints.size(); j++)
            {
                if( skt->joints[j]->nodeId == lst[i] )
                {
                   selMgr.selection.push_back(skt->joints[j]);
                   emit jointDataShow(((joint*)skt->joints[j])->expansion,
                                      ((joint*)skt->joints[j])->nodeId);
                }
            }

        }
    }

}

/*
 void GLWidget::loadSelectVertexCombo(MyMesh& cage)
 {
     QString vertName("Vertice %1");
     parent->ui->selectElementComboLabel->setText("Vertices de la caja");

     MyMesh::VertexIterator vi;
     int idx = 0;
     for(vi = cage.vert.begin(); vi!=cage.vert.end(); ++vi )
         parent->ui->selectElementCombo->addItem(vertName.arg(idx), QVariant(idx++));

     connect(parent->ui->selectElementCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(changeVertexSelection(int)));
 }

 void GLWidget::changeVertexSelection(int id)
 {
     influenceDrawingIdx = parent->ui->selectElementCombo->itemData(id).toInt();
     updateGL();
 }
 */

 bool GLWidget::updateInfo()
 {
     printf("Info function TODO\n"); fflush(0);
     /*
     QString texto = "<b>Escena:</b><br>";
     texto.append(QString("<br><b>-:: Modelo ::-</b><br>%1 vertices<br> %2 triangulos<br>").arg(m.modeloOriginal.vn).arg(m.modeloOriginal.fn));
     texto.append(QString("<br><b>-:: Caja ::-</b><br>%1 vertices<br> %2 triangulos<br>").arg(m.cage.vn).arg(m.cage.fn));
     parent->ui->infoData->setText(texto);

     */

    return true;
 }

 void GLWidget::testScene()
 {
     /*
     skeleton* skt = new skeleton();

     skt->root = new joint(escena->getNewId());
     skt->root->sName = "root";
     skt->root->father = NULL;

     joint* jt = new joint(skt->root, escena->getNewId());
     jt->sName = "jt1";
     skt->root->childs.push_back(jt);
     jt->addTranslation(12,0,0);

     joint* jt2 = new joint(jt, escena->getNewId());
     jt2->sName = "jt2";
     jt->childs.push_back(jt2);
     //jt2->addRotation(45,0,0);
     jt2->addTranslation(0,10,0);

     joint* jt3 = new joint(jt, escena->getNewId());
     jt3->sName = "jt3";
     jt->childs.push_back(jt3);
     jt3->addTranslation(4,0,0);

     joint* jt4 = new joint(jt3, escena->getNewId());
     jt4->sName = "jt4";
     jt3->childs.push_back(jt4);
     jt4->addTranslation(8,0,0);

     joint* jt5 = new joint(jt2, escena->getNewId());
     jt5->sName = "jt5";
     jt2->childs.push_back(jt5);
     jt5->addTranslation(8,0,0);


     escena->skeletons.push_back((object*)skt);

     //ReBuildScene();
     updateInfo();
     emit updateSceneView();
     */
 }



 void GLWidget::PropFunctionConf()
 {
     // TODEBUG
     printf("Ojo!, ahora estamos teniendo en cuenta: 1 solo modelo, esqueleto y grid.\n"); fflush(0);
     Modelo* m = (Modelo*)escena->models.back();
     gridRenderer* grRend = (gridRenderer*)escena->visualizers.back();

     if(m == NULL || grRend == NULL) return;

     // He cambiaso el grid por la maya... hay quye hacer los cambios pertinentes.
     assert(false);
     // TODO

     /*
     // Cogemos los valores del usuario
     grRend->grid->kValue = parent->ui->kvalue_in->text().toFloat();
     grRend->grid->alphaValue = parent->ui->alphavalue_in->text().toFloat();
     grRend->grid->metricUsed = parent->ui->metricUsedCheck->isChecked();

	 // limpiar los pesos calculados anteriormente
	 cleanWeights(grRend);

	 // recalcular los pesos
	 computeHierarchicalSkinning(*(grRend->grid));

	 grRend->propagateDirtyness();
	 updateGridRender();
     */
 }

float GLWidget::calculateDistancesForISOLines(grid3d* grid, vector<double>&  embeddedPoint)
{
	float maxDistance = 0;
	for(int i = 0; i < grid->cells.size(); i++)
	{
		for(int j = 0; j < grid->cells[i].size(); j++)
		{
			for(int k = 0; k < grid->cells[i][j].size(); k++)
			{
				cell3d* cell = grid->cells[i][j][k];

				if(cell->getType() == EXTERIOR || cell->getType() == INTERIOR) continue;

				cell->data->distanceToPos = distancesFromEbeddedPoints(embeddedPoint, cell->data->embedding);
				if(cell->data->distanceToPos > maxDistance)
					maxDistance = cell->data->distanceToPos;
			}
		}
	}

	return maxDistance;
}

 void GLWidget::paintGrid(gridRenderer* grRend)
 {

	float maxDistance = 0;
	if( grRend->iVisMode == VIS_ISODISTANCES || grRend->iVisMode == VIS_ISOLINES)
	{
		// Calcular distancia maxima.
		int findIndex = -1;
		for(int i = 0; i< grRend->grid->v.embeddedPoints.size(); i++)
		{
			if(grRend->grid->v.intPoints[i].boneId == grRend->desiredVertex)
			{
				findIndex = i;
				break;
			}
		}

		if(findIndex >= 0)
		{
			vector<double>& embeddedPoint = grRend->grid->v.embeddedPoints[findIndex];
			maxDistance = calculateDistancesForISOLines(grRend->grid, embeddedPoint);
			//maxDistance = 4000;
		}
		if(maxDistance <= 0)
			maxDistance = 1;
	}

	for(int i = 0; i < grRend->grid->cells.size(); i++)
	{
		for(int j = 0; j < grRend->grid->cells[i].size(); j++)
		{
			for(int k = 0; k < grRend->grid->cells[i][j].size(); k++)
			{
				int newValue;
				float colorValue;
				int idx;
				float nInterval;
				float dif;
				float confidenceLimit;

				cell3d* cell = grRend->grid->cells[i][j][k];
				if(cell->getType() == EXTERIOR || cell->getType() == INTERIOR) continue;

				switch(grRend->iVisMode)
				{
				case VIS_SEGMENTATION:
						newValue = (cell->data->segmentId * 25) % (int)ceil(grRend->grid->valueRange);
						colorValue = newValue/grRend->grid->valueRange;
					break;
				case VIS_BONES_SEGMENTATION:
					    newValue = grRend->grid->v.nodeIds[cell->data->segmentId]->boneId;
						newValue = (newValue * 25) % (int)ceil(grRend->grid->valueRange);
						colorValue = newValue/grRend->grid->valueRange;

						//if(!cell->data->validated)
						//	colorValue = -1;
					break;
				case VIS_SEG_PASS:
						if(cell->data->ownerLabel>0 && cell->data->ownerLabel< grRend->grid->valueRange)
						{
							newValue = grRend->grid->v.nodeIds[cell->data->ownerLabel]->boneId;
							newValue = (newValue * 13) % (int)ceil(grRend->grid->valueRange);
							colorValue = newValue/grRend->grid->valueRange;
						}
						else
						{
							colorValue = 1;
						}
					break;
				case VIS_WEIGHTS:
						idx = findWeight(cell->data->influences, grRend->desiredVertex);

						if(idx >= 0)
							colorValue = cell->data->influences[idx].weightValue;
						else
							colorValue = -1;
				break;
				case VIS_ISODISTANCES:
						colorValue = cell->data->distanceToPos / maxDistance;
					break;

				case VIS_ISOLINES:
						nInterval = maxDistance/20.0;
						dif = cell->data->distanceToPos/nInterval;
						dif = dif*nInterval - floor(dif)*nInterval;
						if( dif < maxDistance/128)
						{
							colorValue = -1;
						}
						else
						{
							colorValue = cell->data->distanceToPos / maxDistance;
						}
					break;

				case VIS_CONFIDENCE_LEVEL:
						colorValue = cell->data->confidenceLevel;
						confidenceLimit = grRend->grid->minConfidenceLevel;
						if(colorValue > confidenceLimit )
							colorValue = confidenceLimit;
						if(colorValue < 0)
							colorValue = 0;
						colorValue = colorValue/confidenceLimit;
					break;

				default:
						colorValue = 1;
					break;
				}

				// En escala de grises para parecernos mas a maya
				float r,g,b;
				if(colorValue < 0)
				{
					cell->data->color[0] = 0;
					cell->data->color[1] = 0;
					cell->data->color[2] = 0;
				}
				else if(colorValue > 1)
				{
					cell->data->color[0] = 1.0;
					cell->data->color[1] = 1.0;
					cell->data->color[2] = 1.0;				
				}
				else
				{
					GetColour(colorValue,0,1,r,g,b);
					cell->data->color[0] = r;
					cell->data->color[1] = g;
					cell->data->color[2] = b;
				}

			}	
		}		
	}
 }


void GLWidget::setPlaneData(bool drawPlane, int pointPos, int mode, float sliderPos, int orient)
{
	for(int i = 0; i< escena->visualizers.size(); i++)
	{
		if(escena->visualizers[i]->iam == PLANERENDERER_NODE)
		{
			clipingPlaneRenderer* planeRender;
			planeRender = (clipingPlaneRenderer*)escena->visualizers[i];
			planeRender->selectedPoint = pointPos;
			planeRender->vmode = (visualizatoinMode)mode;
			planeRender->visible = drawPlane;


			planeRender->position = sliderPos;
			planeRender->posUpdated = false;

			if(orient == 0) // X
			{
				double length = (planeRender->maxPoint[0]-planeRender->minPoint[0]);
				planeRender->plane->pos = planeRender->minPoint + Point3d(length*sliderPos,0,0);
			}
			else if(orient == 1) // Y
			{
				double length = (planeRender->maxPoint[1]-planeRender->minPoint[1]);
				planeRender->plane->pos = planeRender->minPoint + Point3d(0,length*sliderPos,0);
			}
			else if(orient == 2) // Z
			{
				double length = (planeRender->maxPoint[2]-planeRender->minPoint[2]);
				planeRender->plane->pos = planeRender->minPoint + Point3d(0,0,length*sliderPos);
			}

			planeRender->dirtyFlag = true;
		}
	}
}

void GLWidget::paintPlaneWithData(bool compute)
{
	Modelo& m = *((Modelo*)escena->models[0]);

	clipingPlaneRenderer* planeRender = NULL;
	for(int i = 0; i< escena->visualizers.size(); i++)
	{
		if(escena->visualizers[i]->iam == PLANERENDERER_NODE)
		{
			planeRender = (clipingPlaneRenderer*)escena->visualizers[i];
			break;
		}
	}

	m.setSpotVertex(escena->desiredVertex);

	int subdivisions = 50;
	if(!planeRender)
	{
		escena->visualizers.push_back(new clipingPlaneRenderer(escena->getNewId()));
		planeRender = (clipingPlaneRenderer*)escena->visualizers.back();

		Point3d AxisX(1,0,0);
		Point3d AxisY(0,1,0);
		Point3d AxisZ(0,0,1);

		double length = (m.maxBBox - m.minBBox).Norm();
		Point3d center = (m.maxBBox + m.minBBox)/2.0;
		planeRender->minPoint = Point3d(center[0]-length/2,center[1]-length/2,center[2]-length/2);
		planeRender->maxPoint = Point3d(center[0]+length/2,center[1]+length/2,center[2]+length/2);
		planeRender->plane->addTranslation(center[0]-length/2,center[1]-length/2,center[2]-length/2);
		planeRender->subdivisions = subdivisions;
		planeRender->points.resize(subdivisions*subdivisions);

		vector<Point3d>& points = planeRender->points; 
		double pass = length/subdivisions;
		for(unsigned int planoX = 0; planoX < subdivisions; planoX++)
		{
			for(unsigned int planoZ = 0; planoZ < subdivisions; planoZ++)
			{
				points[planoX*subdivisions+planoZ] = Point3d(planoX*pass, planoZ*pass, 0);
			}
		}
	}
	vector<Point3f>& colors = planeRender->colors;
	colors.resize(planeRender->points.size());

	vector< vector<double> >& weights = planeRender->weights;
	if(compute || planeRender->dirtyFlag)
	{
		weights.resize(planeRender->points.size());

		int bindingIdx = 0;
		for(int i = 0; i < planeRender->points.size(); i++)
		{
			//mvcSingleBinding(planeRender->points[i]+planeRender->plane->pos, weights[i], m.bindings[0], m);
			mvcAllBindings(planeRender->points[i]+planeRender->plane->pos, weights[i], m.bindings, m);
		}
	}

	// Ordenamos los pesos de cada punto del plano
	vector< vector<int> >weightsSort(weights.size());
	for(int i = 0; i < weights.size(); i++) weightsSort[i].resize(m.vn());
	vector< vector<bool> > weightsRepresentative(weights.size());
	for(int i = 0; i < weights.size(); i++) weightsRepresentative[i].resize(m.vn());
	double threshold = -10;//1/pow(10.0, 3);

	for(int i = 0; i < weights.size(); i++)
		doubleArrangeElements(weights[i], weightsSort[i], false, threshold);

	if(planeRender->vmode == VIS_WEIGHTS_ONPLANE)
	{
		double minValue = 1;
		double maxValue = 0;
		for(unsigned int planoX = 0; planoX < planeRender->subdivisions; planoX++)
		{
			for(unsigned int planoZ = 0; planoZ < planeRender->subdivisions; planoZ++)
			{
				maxValue = max(weights[planoX*planeRender->subdivisions+planoZ][m.globalIndirection[escena->desiredVertex]], maxValue);
				minValue = min(weights[planoX*planeRender->subdivisions+planoZ][m.globalIndirection[escena->desiredVertex]], minValue);
			}
		}

		printf("Valor de peso min: %f\n",minValue); fflush(0);
		printf("Valor de peso max: %f\n",maxValue); fflush(0);

		for(unsigned int planoX = 0; planoX < planeRender->subdivisions; planoX++)
		{
			for(unsigned int planoZ = 0; planoZ < planeRender->subdivisions; planoZ++)
			{
				float r,g,b;
				double weightAux = weights[planoX*planeRender->subdivisions+planoZ][m.globalIndirection[escena->desiredVertex]];
				
				//GetColour(weightAux, minValue, maxValue, r, g, b);
				GetColourGlobal(weightAux, minValue, maxValue, r, g, b);
				colors[planoX*planeRender->subdivisions+planoZ] = Point3f(r,g,b);
			}
		}

		planeRender->posUpdated = true;
	}
	else if(planeRender->vmode == VIS_DISTANCES)
	{
		vector<double> pointDistances;
		pointDistances.resize(planeRender->points.size());
		double maxDistance = 0;
		for(int planePoint = 0; planePoint < planeRender->points.size(); planePoint++)
		{
			double preValue = 0;
			preValue = PrecomputeDistancesSingular_sorted(weights[planePoint], weightsSort[planePoint], m.bindings[0]->BihDistances, threshold);

			// solo calculamos hacia un vertice de la maya
			pointDistances[planePoint] = -BiharmonicDistanceP2P_sorted(weights[planePoint], weightsSort[planePoint], escena->desiredVertex , m.bindings[0], 1.0, preValue, threshold);
			maxDistance = max(pointDistances[planePoint], maxDistance);
		}

		for(unsigned int planoX = 0; planoX < planeRender->subdivisions; planoX++)
		{
			for(unsigned int planoZ = 0; planoZ < planeRender->subdivisions; planoZ++)
			{
				float r,g,b;
				double distance = pointDistances[planoX*planeRender->subdivisions+planoZ];
				
				//GetColour(weightAux, minValue, maxValue, r, g, b);
				GetColourGlobal(distance, 0, maxDistance, r, g, b);
				colors[planoX*planeRender->subdivisions+planoZ] = Point3f(r,g,b);
			}
		}
	}
}

void GLWidget::paintModelWithData()
 {
    for(unsigned int modelIdx = 0; modelIdx < escena->models.size(); modelIdx++)
    {
		Modelo* m = (Modelo*)escena->models[modelIdx];

		if(m->shading->colors.size() != m->vn())
			m->shading->colors.resize(m->vn());

		m->cleanSpotVertexes();

		double maxDistance = 0.001;
		vector<double> pointDistances;
		vector<double> maxDistances;
		maxDistances.resize(m->bindings.size());

		vector<double> weights;
		vector<int> weightsSort(m->vn());
		vector<bool> weightsRepresentative(m->vn());

		vector<double> completeDistances;
		double threshold = escena->weightsThreshold;//1/pow(10.0, 3);

		if(escena->iVisMode == VIS_ISODISTANCES || escena->iVisMode == VIS_POINTDISTANCES || escena->iVisMode == VIS_ERROR || escena->iVisMode == VIS_WEIGHT_INFLUENCE)
		{
			printf("Tiempos de computacion para %d vertices: \n", m->vn()); fflush(0);
			/*for(int currentBinding = 0; currentBinding < m->bindings.size(); currentBinding++)
			{		
				for(int i = 0; i< m->bindings[currentBinding]->BihDistances.size; i++)
				{
					for(int j = i; j< m->bindings[currentBinding]->BihDistances.size; j++)
					{
						maxDistance = max(m->bindings[currentBinding]->BihDistances.get(i,j), maxDistance);
					}
				}
			}
			maxDistance *= 2;*/
			
			clock_t ini, fin;
			ini = clock();

			weights.resize(m->vn());
			int currentBinding = 1;
			pointDistances.resize(m->vn());
			//pointDistances.resize(m->bindings[currentBinding]->pointData.size());
			//mvcSingleBinding(interiorPoint, weights, m->bindings[currentBinding], *m);
			mvcAllBindings(interiorPoint, weights, m->bindings, *m);

			fin = clock();
			printf("Mean Value Coordinates: %f ms\n", timelapse(fin,ini)*1000); fflush(0);
			ini = clock();

			//for(int si = 0; si < weights.size(); si++)
			//{
			//	weightsSort[si] = si;
			//}
			//doubleArrangeElements(weights, weightsSort, true, threshold);
			vector<double> stats;
			doubleArrangeElements_withStatistics(weights, weightsSort, stats, threshold);


			fin = clock();
			printf("Ordenacion: %f ms\n", timelapse(fin,ini)*1000); fflush(0);
			ini = clock();

			double preValue = 0;
			for(int currentBinding = 0; currentBinding < m->bindings.size(); currentBinding++)
			{
				int iniIdx = m->bindings[currentBinding]->globalIndirection.front();
				int finIdx = m->bindings[currentBinding]->globalIndirection.back();
				preValue += PrecomputeDistancesSingular_sorted(weights, weightsSort,  m->bindings[currentBinding]->BihDistances, threshold);
			}

			fin = clock();
			printf("Precomputo: %f ms\n", timelapse(fin,ini)*1000); fflush(0);
			ini = clock();

			for(int i = 0; i< m->modelVertexBind.size(); i++)
			{
				int iBind = m->modelVertexBind[i];
				int iniIdx = m->bindings[iBind]->globalIndirection.front();
				int finIdx = m->bindings[iBind]->globalIndirection.back();

				int iVertexBind = m->modelVertexDataPoint[i];

				pointDistances[i] = -BiharmonicDistanceP2P_sorted(weights, weightsSort, iVertexBind , m->bindings[iBind], 1.0, preValue, threshold);

				maxDistances[iBind] = max(maxDistances[iBind],pointDistances[i]);
				maxDistance = max(pointDistances[i], maxDistance);
			}

			fin = clock();
			printf("Calculo distancias: %f ms\n", timelapse(fin,ini)*1000); fflush(0);
			ini = clock();

			for(int md = 0; md< maxDistances.size(); md++)
			{
				printf("Bind %d con maxDist: %f\n", md, maxDistances[md]);
				fflush(0);
			}

			if(escena->iVisMode == VIS_ERROR)
			{
				completeDistances.resize(m->vn());
				double preValue2 = 0;
				for(int currentBinding = 0; currentBinding < m->bindings.size(); currentBinding++)
				{
					int iniIdx = m->bindings[currentBinding]->globalIndirection.front();
					int finIdx = m->bindings[currentBinding]->globalIndirection.back();
					preValue2 += PrecomputeDistancesSingular_sorted(weights, weightsSort,  m->bindings[currentBinding]->BihDistances, -10);
				}

				for(int i = 0; i< m->modelVertexBind.size(); i++)
				{
					int iBind = m->modelVertexBind[i];
					int iniIdx = m->bindings[iBind]->globalIndirection.front();
					int finIdx = m->bindings[iBind]->globalIndirection.back();

					int iVertexBind = m->modelVertexDataPoint[i];
					completeDistances[i] = -BiharmonicDistanceP2P_sorted(weights, weightsSort, iVertexBind , m->bindings[iBind], 1.0, preValue2, -10);
				}
			}
		}

		double maxError = -9999;
		if(m->bindings.size() <= 0) continue;
		for(int currentBinding = 0; currentBinding < m->bindings.size(); currentBinding++)
		{
			for(int count = 0; count< m->bindings[currentBinding]->pointData.size(); count++)
			{	
				if(m->bindings[currentBinding]->pointData[count].isBorder)
					m->addSpotVertex(m->bindings[currentBinding]->pointData[count].modelVert);

				float value = 0.0;
				// Deberia ser al reves, recorrer el binding y este pinta los puntos del modelo. 
				// Hacer un reset antes con el color propio del modelo.
				binding* bd = m->bindings[currentBinding];
				PointData& pd = bd->pointData[count];
				int newValue = 0;
				if(escena->iVisMode == VIS_LABELS)
				{
					value = (float)pd.component/(float)m->bindings.size();
				}
				else if(escena->iVisMode == VIS_SEGMENTATION)
				{
					newValue = (pd.segmentId-100)*13;
					value = ((float)(newValue%50))/50.0;
				}
				else if(escena->iVisMode == VIS_BONES_SEGMENTATION)
				{
					newValue = (bd->nodeIds[pd.segmentId]->boneId-100)*13;
					//newValue = (newValue * 25) % 100;
					value = ((float)(newValue%25))/25.0;
				}
				else if(escena->iVisMode == VIS_WEIGHTS)
				{
					//float sum = 0; 
					value = 0.0;

					int searchedIndex = -1;
					for(unsigned int ce = 0; ce < pd.influences.size(); ce++)
					{
						if(pd.influences[ce].label == escena->desiredVertex)
						{
							searchedIndex = ce;
							break;
						}
						//sum += pd.influences[ce].weightValue;
					}
					if(searchedIndex >= 0)
							value = pd.influences[searchedIndex].weightValue;

					//if (sum < 1) 
					//	printf("No se cumple la particion de unidad: %f.\n", sum);
				}
				else if(escena->iVisMode == VIS_SEG_PASS)
				{
					value = 0.0;
					if(bd->nodeIds[pd.segmentId]->boneId == escena->desiredVertex)
					{
						value = 1.0;
					}
				}
				else if(escena->iVisMode == VIS_CONFIDENCE_LEVEL)
				{
					value = 0.0;
					if(count == escena->desiredVertex)
					{
						value = 1.0;
					}
					else
					{
						for(int elemvecino = 0; elemvecino < bd->surface.nodes[count]->connections.size() ; elemvecino++)
						{
							if(bd->surface.nodes[count]->connections[elemvecino]->id == escena->desiredVertex)
							{
								value = 0.5;
								break;
							}
						}						
					}
				}
				else if(escena->iVisMode == VIS_ISODISTANCES)
				{
					if(escena->desiredVertex <0 || escena->desiredVertex >= bd->pointData.size()) 
						value = 0;
					else
					{
						if(maxDistance <= 0)
							value = 0;
						else
							value = m->bindings[currentBinding]->BihDistances.get(count,escena->desiredVertex) / maxDistance;
					}
				}
				else if(escena->iVisMode == VIS_POINTDISTANCES)
				{
					if(maxDistance <= 0)
						value = 0;
					else
					{
						int modelVert = m->bindings[currentBinding]->pointData[count].modelVert;
						value = pointDistances[modelVert] / maxDistances[currentBinding];
					}
				}
				else if(escena->iVisMode == VIS_ERROR)
				{
					int modelVert = m->bindings[currentBinding]->pointData[count].modelVert;
					
					if(completeDistances[modelVert] > 0)
						value = pointDistances[modelVert] - completeDistances[modelVert];

					maxError = max((double)maxError, (double)value);

				}
				else if(escena->iVisMode == VIS_WEIGHT_INFLUENCE)
				{
					int modelVert = m->bindings[currentBinding]->pointData[count].modelVert;
					value = weights[modelVert];
					//if(maxDistance <= 0)
					//	value = 0;
					//else
					//{
					//	int modelVert = m->bindings[currentBinding]->pointData[count].modelVert;
					//	value = pointDistances[modelVert] / maxDistances[currentBinding];
					//}
				}
				else
				{
					value = 1.0;
				}

				float r,g,b;
				GetColourGlobal(value,0.0,1.0, r, g, b);
				//QColor c(r,g,b);
				m->shading->colors[pd.modelVert].resize(3);
				m->shading->colors[pd.modelVert][0] = r;
				m->shading->colors[pd.modelVert][1] = g;
				m->shading->colors[pd.modelVert][2] = b;
			}
		}

		printf("Corte:%f\n", threshold); fflush(0);
		printf("Error max:%f\n", maxError); fflush(0);
		
	}
 }

 void GLWidget::paintModelWithGrid()
 {
	/*
    if(VERBOSE)
        printf(">> PaintModelWithGrid (for testing)\n");

    // Cada grid renderer actualizara sus modelos.
    for(unsigned int vis = 0; vis < escena->visualizers.size(); vis++)
    {
        gridRenderer* grRend = (gridRenderer*)escena->visualizers[vis];
        if(grRend == NULL) continue;

        Modelo* m = grRend->model;
        if(m == NULL) continue;

        if(grRend == NULL || !grRend->Initialized) return;

        m->shading->colors.resize(m->vn());

        if(grRend->iVisMode == VIS_SEGMENTATION || grRend->iVisMode == VIS_WEIGHTS)
        {
            int count = 0;
            MyMesh::VertexIterator vi;
            for(vi = m->vert.begin(); vi!=m->vert.end(); ++vi ) {
                m->shading->colors[count].resize(3);
                Point3d pt = vi->P();

                Point3i idx = grRend->grid->cellId(pt);

                float value = 0;

                if(grRend->iVisMode == VIS_WEIGHTS)
                {
                    int searchedIndex = -1;
                    for(unsigned int ce = 0; ce < grRend->grid->cells[idx.X()][idx.Y()][idx.Z()]->data->influences.size(); ce++)
                    {
                        if(grRend->grid->cells[idx.X()][idx.Y()][idx.Z()]->data->influences[ce].label == grRend->desiredVertex)
                        {
                            searchedIndex = ce;
                            break;
                        }
                    }

                    if(searchedIndex >= 0)
                        value = grRend->grid->cells[idx.X()][idx.Y()][idx.Z()]->data->influences[searchedIndex].weightValue;

                }
                else if(grRend->iVisMode == VIS_SEGMENTATION)
                {
                    if(grRend->grid->cells[idx.X()][idx.Y()][idx.Z()]->data->segmentId >= 0 &&
                       grRend->grid->cells[idx.X()][idx.Y()][idx.Z()]->data->segmentId < grRend->grid->valueRange)
					{
						int newValue = (grRend->grid->cells[idx.X()][idx.Y()][idx.Z()]->data->segmentId * 13) % (int)ceil(grRend->grid->valueRange);
                        value = newValue/grRend->grid->valueRange;
					}
					else value = 0;
                }

                assert((grRend->grid->valueRange-1) >= 0);
				float r,g,b;
                GetColour(value,0,1, r, g, b);
				QColor c(r,g,b);
                m->shading->colors[count][0] = c.redF();
                m->shading->colors[count][1] = c.blueF();
                m->shading->colors[count][2] = c.greenF();

                count++;
            }
        }
    }
	*/
 }


 // Lee una lista de puntos.
 
 bool GLWidget::readNodes(vector< string >& nodeNames,
                          vector< Point3d >& nodePoints,
                          QString sFile)
 {
     nodePoints.clear();
     nodeNames.clear();

     QFile inFile(sFile);
     if(!inFile.exists()) return false;

     inFile.open(QFile::ReadOnly);
     QTextStream in(&inFile);
     unsigned int idxNumber;
     idxNumber= in.readLine().toInt();

     nodePoints.resize(idxNumber);
     nodeNames.resize(idxNumber);

     int element = 0;

     while(!in.atEnd())
     {
         QString str = in.readLine();

         QStringList lstr = str.split(" ");

         nodeNames[element] = lstr[0].toStdString();

         Point3d pt;
         for(int i = 0; i< 3; i++)
             pt[i] = lstr[i+1].toDouble();

         nodePoints[element] = pt;

         element++;
     }

     inFile.close();

     return true;
 }


 // Lee una lista de puntos.
 bool GLWidget::readPoints(vector< Point3d >& points,
                           QString sFile)
 {
     points.clear();

     QFile inFile(sFile);
     if(!inFile.exists()) return false;

     inFile.open(QFile::ReadOnly);
     QTextStream in(&inFile);
     unsigned int idxNumber;
     idxNumber= in.readLine().toInt();

     while(!in.atEnd())
     {
         QString str = in.readLine();

         QStringList lstr = str.split(" ");
         assert(lstr.size() == 3);

         Point3d pt;
         for(int i = 0; i< lstr.size(); i++)
             pt[i] = lstr[i].toDouble();

         points.push_back(pt);
     }

     inFile.close();

     assert(points.size() == idxNumber);
     return points.size() == idxNumber;
 }

 
 void GLWidget::exportWeightsToMaya()
 {
	 assert(false);
	 // esta vinculado al grid

	 /*
    gridRenderer* grRend = (gridRenderer*)escena->visualizers.back();
    Modelo* m = (Modelo*)escena->models.back();
    skeleton* skt = (skeleton*)escena->skeletons.back();

    vector< vector<float> > meshWeights;

    meshWeights.resize(m->vn());

    int idxCounter = 0;
    MyMesh::VertexIterator vi;
    for(vi = m->vert.begin(); vi!=m->vert.end(); ++vi )
    {
        Point3i pt = grRend->grid->cellId((*vi).P());
        cell3d* cell = grRend->grid->cells[pt.X()][pt.Y()][pt.Z()];

        meshWeights[idxCounter].resize(grRend->grid->valueRange, 0.0);

        for(unsigned int w = 0; w< cell->data->influences.size(); w++)
        {
            meshWeights[idxCounter][cell->data->influences[w].label] =  cell->data->influences[w].weightValue;
        }

        idxCounter++;
    }

    vector<string> names;
    skt->GetJointNames(names);

    string fileName = QString("%1%2.weights").arg(m->sPath.c_str()).arg(m->sName.c_str()).toStdString();
    saveWeightsToMayaByName(m, meshWeights, names, fileName);
	*/
 }

void GLWidget::setThreshold(double value)
{
	if(value == -10)
		escena->weightsThreshold = -10;
	else if(value == 0)
		escena->weightsThreshold = 0;
	else
		escena->weightsThreshold = 1/pow(10,value);
	paintModelWithData();
}

 void GLWidget::VoxelizeModel(Modelo* m, bool onlyBorders)
 {
     /*
	 // Hemos vinculado el grid al modelo y esqueleto cargados.
	 // Hay que gestionar esto con cuidado.
	 printf("Ojo!, usamos los modelos y skeletos tal cual...\n");
     escena->visualizers.push_back((shadingNode*)new gridRenderer());
     gridRenderer* grRend = (gridRenderer*)escena->visualizers.back();
     grRend->iam = GRIDRENDERER_NODE;
     grRend->model = m;

	 // grid creation for computation
	 m->grid = new grid3d();
	 grRend->grid = m->grid;

	 grRend->grid->res = parent->ui->gridResolutionIn->text().toInt();
	 grRend->grid->worldScale = parent->ui->sceneScale->text().toInt();

	 for(int i = 0; i< escena->skeletons.size(); i++)
	 {
		 float cellSize = grRend->grid->cellSize;
		 GetMinSegmentLenght(getMinSkeletonSize((skeleton*)escena->skeletons[i]), cellSize);
		 ((skeleton*)escena->skeletons[i])->minSegmentLength = GetMinSegmentLenght(getMinSkeletonSize((skeleton*)escena->skeletons[i]), cellSize);

		 grRend->grid->bindedSkeletons.push_back((skeleton*)escena->skeletons[i]);
	 }

     clock_t begin, end;

     vector< vector<double> >* embedding = NULL;
     if(m->embedding.size() != 0)
     {
         embedding = &(m->embedding);
     }
     else // No se ha cargado ningun embedding.
     {
         printf("No hay cargado ning�n embedding. Lo siento, no puedo hacer calculos\n"); fflush(0);
         return;
     }

     // Cargamos el grid si ya ha sido calculado
     QString sSavingFile = QString("%1%2%3").arg(m->sPath.c_str()).arg(m->sName.c_str()).arg("_embedded_grid.dat");
     if(!sSavingFile.isEmpty() && QFile(sSavingFile).exists())
     {
         if(VERBOSE)
         {
            cout << "Loading from disc: ";
            begin=clock();
         }

         grRend->grid->LoadGridFromFile(sSavingFile.toStdString());

         if(VERBOSE)
         {
             end = clock();
             cout << double(timelapse(end,begin)) << " s"<< endl;
         }
     }
     else
     {
         if(VERBOSE)
         {
             if(!onlyBorders) cout << "Computing volume: " << endl;
             else cout << "Computing volume (onlyAtBorders): " << endl;

             cout << "------------------------------------------------------" << endl;
             cout << "                  - PREPROCESO -                      " << endl;
             cout << "------------------------------------------------------" << endl;
             cout << "Creaci�n del grid con res " << grRend->grid->res << endl;
             begin=clock();

         }

         // CONSTRUCCION DEL GRID
         gridInit(*m,*(grRend->grid));

         if(VERBOSE)
         {
             end=clock();
             cout << double(timelapse(end,begin)) << " s"<< endl;
             begin = end;
             cout << "Etiquetado del grid: ";
         }

         //Tipificamos las celdas seg�n si es interior, exterior, o borde de la maya
         grRend->grid->typeCells(*m);

         if(VERBOSE)
         {
             end=clock();
             cout << double(timelapse(end,begin)) << " s"<< endl;
             cout << "Grid cells embedding";
             begin=clock();
         }

         // EMBEDING INTERPOLATION FOR EVERY CELL
         int interiorCells = 0;
         interiorCells = gridCellsEmbeddingInterpolation(*m, *(grRend->grid), m->embedding, onlyBorders);

         if(VERBOSE)
         {
             end=clock();
             cout << "("<< interiorCells <<"cells): " << double(timelapse(end,begin)) << " s"<< endl;
             double memorySize = (double)(interiorCells*DOUBLESIZE*embedding[0].size())/MBSIZEINBYTES;
             cout << "Estimated memory consumming: " << memorySize << "MB" <<endl;
             cout << ">> TOTAL (construccion del grid desde cero): "<<double(timelapse(end,begin)) << " s"<< endl;

         }

         if(!sSavingFile.isEmpty())
         {
             if(VERBOSE)
             {
                cout << "Guardando en disco: ";
                begin=clock();
             }

             grRend->grid->SaveGridToFile(sSavingFile.toStdString());

             if(VERBOSE)
             {
                 clock_t end=clock();
                 cout << double(timelapse(end,begin)) << " s"<< endl;
             }
         }
     }

     grRend->propagateDirtyness();
     updateGridRender();
     */
 }

void GLWidget::allNextProcessSteps()
{
    /*
	for(int i = 0; i< CurrentProcessJoints.size(); i++)
	{
		computeHierarchicalSkinning(*(currentProcessGrid->grid), CurrentProcessJoints[i]);
	}
	CurrentProcessJoints.clear();
    */
}

void GLWidget::nextProcessStep()
{
    /*
	if(CurrentProcessJoints.size() > 0)
	{
		joint* jt = CurrentProcessJoints[0];
		for(int i = 1; i< CurrentProcessJoints.size(); i++)
		{
			CurrentProcessJoints[i-1] = CurrentProcessJoints[i];
		}
		
		CurrentProcessJoints.pop_back();

		computeHierarchicalSkinning(*(currentProcessGrid->grid), jt);
	}
    */
}


void loadDataForDrawing(gridRenderer* grRend )
{
	int borderCounter = 0;
	for(unsigned int i = 0; i< grRend->grid->cells.size(); i++)
	{
		for(unsigned int j = 0; j< grRend->grid->cells[i].size(); j++)
		{
			for(unsigned int k = 0; k< grRend->grid->cells[i][j].size(); k++)
			{
				cell3d* cell = grRend->grid->cells[i][j][k];
				if(cell->getType() == BOUNDARY)
				{
					borderCounter++;
				}
			}
		}
	}
	
	grRend->grid->borderCellsCounts = borderCounter;
}


void reportResults(Modelo& model, binding* bb)
{
	assert(false);
	 // esta vinculado al grid

	/*
	
	//printf("getting results %d points and %d nodes.\n",weights.points, weights.nodes);	
	int count = 0;
	float sum = 0;
	MyMesh::VertexIterator vi;

//	grid3d& grid = *(model->grid);

	printf("Hola"); fflush(0); 

	FILE* fout = fopen("secondaryWeights.txt", "w");
    for(vi = model.vert.begin(); vi!=model.vert.end(); ++vi )
	{

        // Obtener valores directamente de la estructura de datos del modelo.
        //TODO
        assert(false);

		// Obtener celda
		Point3d ptPos = vi->P();
		Point3i pt = grid.cellId(ptPos);
		cell3d* cell = grid.cells[pt.X()][pt.Y()][pt.Z()];

		fprintf(fout, "Influences: %d -> ", cell->data->influences.size()); fflush(fout);

		// copiar pesos
		for(unsigned int w = 0; w< cell->data->influences.size(); w++)
		{
			float auxWeight = cell->data->influences[w].weightValue;
			int idDeformer = cell->data->influences[w].label;
			//escena->defIds[idDeformer];
			fprintf(fout, "(%d,  %f) ", idDeformer, auxWeight); fflush(fout);
		}
		fprintf(fout, "\n"); fflush(fout);
		fprintf(fout, "Secondary Influences: %d -> ", cell->data->auxInfluences.size()); fflush(fout);
		for(unsigned int w = 0; w< cell->data->auxInfluences.size(); w++)
		{
			float auxWeight = cell->data->auxInfluences[w].weightValue;
			int idDeformer = cell->data->auxInfluences[w].label;
			int idRelDeformer = cell->data->auxInfluences[w].relativeLabel;

			//escena->defIds[idDeformer];
			fprintf(fout, "(%d,%d:%f) ", idDeformer,idRelDeformer, auxWeight); fflush(fout);
		}
		fprintf(fout, "\n\n"); fflush(fout);
		count++;
        
	}

	*/
}


/*
void GLWidget::doTestsSkinning(string fileName, string name, string path)
{
	
	// Realizaremos los tests especificados en un fichero.
	QFile file(QString(fileName.c_str()));

	if(!file.open(QFile::ReadOnly)) return;

	QTextStream in(&file);
        
	QString resultFileName = in.readLine();
	QString workingPath = in.readLine();

	QStringList globalCounts = in.readLine().split(" ");
	int modelCount = globalCounts[0].toInt();
	int interiorPointsCount = globalCounts[1].toInt();

	vector<QString> modelPaths(modelCount);
	vector<QString> modelSkeletonFile(modelCount);
	vector<QString> modelNames(modelCount);
	vector< vector<QString> >modelFiles(modelCount);
	vector< vector<Point3d> >interiorPoints(modelCount);

	QStringList thresholdsString = in.readLine().split(" ");
	vector< double >thresholds(thresholdsString.size());
	for(int i = 0; i< thresholds.size(); i++)
		thresholds[i] = thresholdsString[i].toDouble();

	for(int i = 0; i< modelCount; i++)
	{
		modelNames[i] = in.readLine();
		modelPaths[i] = in.readLine();
		modelSkeletonFile[i] = in.readLine();
		modelFiles[i].resize(in.readLine().toInt());

		for(int j = 0; j < modelFiles[i].size(); j++)
			modelFiles[i][j] = in.readLine();

		QStringList intPointsModel = in.readLine().split(" ");
		interiorPoints[i].resize(interiorPointsCount);

		assert(intPointsModel.size() == interiorPointsCount*3);

		for(int j = 0; j< intPointsModel.size(); j = j+3)
		{
			double v1 = intPointsModel[j].toDouble();
			double v2 = intPointsModel[j+1].toDouble();
			double v3 = intPointsModel[j+2].toDouble();
			interiorPoints[i][j/3] = Point3d(v1, v2, v3);
		}
	}
	file.close();

	// Preparamos y lanzamos el calculo.
	QFile outFile(workingPath+"/"+resultFileName);
	if(!outFile.open(QFile::WriteOnly)) return;
	QTextStream out(&outFile);

	// time variables.
	float time_init = -1;
	float time_AComputation = -1;
	float time_MVC_sorting = -1;
	float time_precomputedDistance = -1;
	float time_finalDistance = -1;

	for(int i = 0; i< modelCount; i++)
	{
		for(int mf = 0; mf< modelFiles[i].size(); mf++)
		{
			//Leemos el modelo
			QString file = workingPath+"/"+modelPaths[i]+"/"+modelFiles[i][mf];
			QString path = workingPath+"/"+modelPaths[i]+"/";
			readModel( file.toStdString(), modelNames[i].toStdString(), path.toStdString());
			Modelo* m = ((Modelo*)escena->models.back());

			QString sktFile = workingPath+"/"+modelPaths[i]+"/"+modelSkeletonFile[i];

			clock_t ini, fin;
			ini = clock();

			// Incializamos los datos
			BuildSurfaceGraphs(*m, m->bindings);

			for(int bindSkt = 0; bindSkt < m->bindings.size(); bindSkt++)
				readSkeletons(sktFile.toStdString(), m->bindings[bindSkt]->bindedSkeletons);

			for(int bindSkt = 0; bindSkt < m->bindings.size(); bindSkt++)
			{
				for(int bindSkt2 = 0; bindSkt2 < m->bindings[bindSkt]->bindedSkeletons.size(); bindSkt2++)
				{
					float minValue = GetMinSegmentLenght(getMinSkeletonSize(m->bindings[bindSkt]->bindedSkeletons[bindSkt2]),0);
					(m->bindings[bindSkt]->bindedSkeletons[bindSkt2])->minSegmentLength = minValue;
				}
			}

			fin = clock();
			time_init = timelapse(fin,ini)*1000;
			ini = clock();

			// Cálculo de A
			ComputeEmbeddingWithBD(*m);

			fin = clock();
			time_AComputation = timelapse(fin,ini)*1000;

			vector< vector< vector<double> > >statistics;
			statistics.resize(interiorPoints[i].size());
			vector<float> time_MVC(interiorPoints[i].size(), -1);

			// Guardamos datos
			//vector<vector<vector<float> > >statistics(interiorPoints[i].size());
			//vector<float> > time_MVC(interiorPoints[i].size(), -1);
			out << file+"_results.txt" << endl;
			out << file+"_distances.txt" << endl;
			out << file+"_pesos.txt" << endl;

			// Exportamos la segmentacion y los pesos en un fichero.
			QFile pesosFile(file+"_pesos.txt");
			if(!pesosFile.open(QFile::WriteOnly)) return;
			QTextStream outPesos(&pesosFile);

			// Calculo de pesos
			for(int k = 0; k < thresholds.size(); k++)
			{
				for(int i = 0; i< m->bindings.size(); i++)
				{
					m->bindings[i]->weightsCutThreshold = thresholds[k];
				}

				printf("Computing skinning:\n");

				// Realizamos el calculo para cada binding
				clock_t iniImportante = clock();
				ComputeSkining(*m);
				clock_t finImportante = clock();

				double timeComputingSkinning = timelapse(finImportante,iniImportante);

				outPesos << "Threshold: " << thresholds[k] << " tiempo-> "<< timeComputingSkinning << endl;
				char* charIntro;
				scanf(charIntro);

				for(int bindId = 0; bindId < m->bindings.size(); bindId++)
				{
					for(int pointBind = 0; pointBind < m->bindings[bindId]->pointData.size(); pointBind++)
					{
						PointData& pt = m->bindings[bindId]->pointData[pointBind];
						outPesos << pt.ownerLabel << ", ";
					}

					outPesos << endl;
				}
			}

			pesosFile.close();

			// eliminamos el modelo
			delete m;
			escena->models.clear();
			
		}
	}
	outFile.close();
	
}
*/

void GLWidget::getStatisticsFromData(string fileName, string name, string path)
{
	QFile file(QString(fileName.c_str()));
	if(!file.open(QFile::ReadOnly)) return;
	QTextStream in(&file);

	//Leemos los datos.
	vector<QString> paths;
	vector< vector<QString> > fileNames;

	int numPaths = in.readLine().toInt();
	paths.resize(numPaths);
	for(int i = 0; i < numPaths; i++)
		paths[i] = in.readLine();

	int numFiles = in.readLine().toInt();
	fileNames.resize(numFiles);
	for(int i = 0; i < numFiles; i++)
	{
		int numResolutions = in.readLine().toInt();
		fileNames[i].resize(numResolutions);
		for(int j = 0; j < numResolutions; j++)
		{
			fileNames[i][j] = in.readLine();
		}
	}

	file.close();


	for(int i = 0; i < fileNames.size(); i++)
	{
		for(int j = 0; j < fileNames[i].size(); j++)
		{
			vector< vector<int> >labels;
			labels.resize(paths.size());
			for(int k = 0; k < paths.size(); k++)
			{
				QFile file2(QString(paths[k] + fileNames[i][j]));
				if(!file2.open(QFile::ReadOnly)) 
					continue;

				QTextStream inFile(&file2);

				QString dummy = inFile.readLine();

				int value;
				while(!inFile.atEnd())
				{
					inFile >> value;
					labels[k].push_back(value);
					inFile >> dummy;
				}
				file2.close();
			}

			vector<int> counters;
			counters.resize(labels.size(),0);
			int countTotal;
			for(int l = 1; l< labels.size(); l++)
			{
				countTotal = labels[0].size();
				for(int c = 0; c< labels[0].size(); c++)
				{
					if(labels[0][c] == labels[l][c]) counters[l]++;
				}
			}

			FILE* fout = fopen("C:/DATA/phd_dev/data/models_for_test/stats_aciertos.txt", "a");
			fprintf(fout, "Results: %s\n", fileNames[i][j].toStdString().c_str());
			for(int l = 1; l < labels.size(); l++)
			{
				fprintf(fout, "Aciertos[%d]: %d de %d -> %f\n", l, counters[l], countTotal, (double)counters[l]/(double)countTotal);
			}
			fprintf(fout, "\n");
			fclose(fout);
		}
	}

	file.close();

}

void GLWidget::doTests(string fileName, string name, string path)
{
	getStatisticsFromData(fileName, name, path);
	return;

	// Realizaremos los tests especificados en un fichero.
	QFile file(QString(fileName.c_str()));

	if(!file.open(QFile::ReadOnly)) return;

	QTextStream in(&file);
        
	QString resultFileName = in.readLine();
	QString workingPath = in.readLine();

	QStringList globalCounts = in.readLine().split(" ");
	int modelCount = globalCounts[0].toInt();
	int interiorPointsCount = globalCounts[1].toInt();

	vector<QString> modelPaths(modelCount);
	vector<QString> modelSkeletonFile(modelCount);
	vector<QString> modelNames(modelCount);
	vector< vector<QString> >modelFiles(modelCount);
	vector< vector<Point3d> >interiorPoints(modelCount);

	QStringList thresholdsString = in.readLine().split(" ");
	vector< double >thresholds(thresholdsString.size());
	for(int i = 0; i< thresholds.size(); i++)
		thresholds[i] = thresholdsString[i].toDouble();

	for(int i = 0; i< modelCount; i++)
	{
		modelNames[i] = in.readLine();
		modelPaths[i] = in.readLine();
		modelSkeletonFile[i] = in.readLine();
		modelFiles[i].resize(in.readLine().toInt());

		for(int j = 0; j < modelFiles[i].size(); j++)
			modelFiles[i][j] = in.readLine();

		QStringList intPointsModel = in.readLine().split(" ");
		interiorPoints[i].resize(interiorPointsCount);

		assert(intPointsModel.size() == interiorPointsCount*3);

		for(int j = 0; j< intPointsModel.size(); j = j+3)
		{
			double v1 = intPointsModel[j].toDouble();
			double v2 = intPointsModel[j+1].toDouble();
			double v3 = intPointsModel[j+2].toDouble();
			interiorPoints[i][j/3] = Point3d(v1, v2, v3);
		}
	}
	file.close();

	// Preparamos y lanzamos el calculo.
	QFile outFile(workingPath+"/"+resultFileName);
	if(!outFile.open(QFile::WriteOnly)) return;
	QTextStream out(&outFile);

	// time variables.
	float time_init = -1;
	float time_AComputation = -1;
	float time_MVC_sorting = -1;
	float time_precomputedDistance = -1;
	float time_finalDistance = -1;

	for(int i = 0; i< modelCount; i++)
	{
		for(int mf = 0; mf< modelFiles[i].size(); mf++)
		{
			//Leemos el modelo
			QString file = workingPath+"/"+modelPaths[i]+"/"+modelFiles[i][mf];
			QString path = workingPath+"/"+modelPaths[i]+"/";
			readModel( file.toStdString(), modelNames[i].toStdString(), path.toStdString());
			Modelo* m = ((Modelo*)escena->models.back());

			QString sktFile = workingPath+"/"+modelPaths[i]+"/"+modelSkeletonFile[i];

			clock_t ini, fin;
			ini = clock();

			// Incializamos los datos
			BuildSurfaceGraphs(*m, m->bindings);

			for(int bindSkt = 0; bindSkt < m->bindings.size(); bindSkt++)
				readSkeletons(sktFile.toStdString(), m->bindings[bindSkt]->bindedSkeletons);

			for(int bindSkt = 0; bindSkt < m->bindings.size(); bindSkt++)
			{
				for(int bindSkt2 = 0; bindSkt2 < m->bindings[bindSkt]->bindedSkeletons.size(); bindSkt2++)
				{
					float minValue = GetMinSegmentLenght(getMinSkeletonSize(m->bindings[bindSkt]->bindedSkeletons[bindSkt2]),0);
					(m->bindings[bindSkt]->bindedSkeletons[bindSkt2])->minSegmentLength = minValue;
				}
			}

			fin = clock();
			time_init = timelapse(fin,ini)*1000;
			ini = clock();

			// Cálculo de A
			ComputeEmbeddingWithBD(*m);

			fin = clock();
			time_AComputation = timelapse(fin,ini)*1000;

			vector< vector< vector<double> > >statistics;
			statistics.resize(interiorPoints[i].size());
			vector<float> time_MVC(interiorPoints[i].size(), -1);

			QFile distFile(file+"_distances.txt");
			if(!distFile.open(QFile::WriteOnly)) return;
			QTextStream outDistances(&distFile);

			for(int j = 0; j< interiorPoints[i].size(); j++)
			{		
				// Calculo de distancias
				vector<double> pointDistances;
				vector<double> weights;
				weights.resize(m->vn());
				pointDistances.resize(m->vn());
				statistics[j].resize(thresholds.size());

				ini = clock();

				mvcAllBindings(interiorPoints[i][j], weights, m->bindings, *m);

				fin = clock();
				time_MVC[j] = timelapse(fin,ini)*1000;

				outDistances << "Point " << j << endl;
			
				for(int k = 0; k < thresholds.size(); k++)
				{
					vector<int> weightsSort(m->vn());
					vector<bool> weightsRepresentative(m->vn());

					double currentTh = thresholds[k];

					ini = clock();
					doubleArrangeElements_withStatistics(weights, weightsSort, statistics[j][k], currentTh);

					/* 8 values
					statistics[0] = weights.size();
					statistics[1] = minValue;
					statistics[2] = maxValue;
					statistics[3] = sumValues;
					statistics[4] = countOutValues;
					statistics[5] = sumOutValues;
					statistics[6] = countNegValues;
					statistics[7] = sumNegValues;
					*/

					fin = clock();
					time_MVC_sorting = timelapse(fin,ini)*1000;
					statistics[j][k].push_back(time_MVC_sorting);
					ini = clock();

					double preValue = 0;
					for(int currentBinding = 0; currentBinding < m->bindings.size(); currentBinding++)
					{
						int iniIdx = m->bindings[0]->globalIndirection.front();
						int finIdx = m->bindings[0]->globalIndirection.back();
						preValue += PrecomputeDistancesSingular_sorted(weights, weightsSort,  m->bindings[currentBinding]->BihDistances, currentTh);
					}

					fin = clock();
					time_precomputedDistance = timelapse(fin,ini)*1000;
					statistics[j][k].push_back(time_precomputedDistance);
					ini = clock();
			
					outDistances << "Threshold " << currentTh << endl ;

					double maxDistance = -1;
					for(int i = 0; i< m->modelVertexBind.size(); i++)
					{
						int iBind = m->modelVertexBind[i];
						int iniIdx = m->bindings[iBind]->globalIndirection.front();
						int finIdx = m->bindings[iBind]->globalIndirection.back();

						int iVertexBind = m->modelVertexDataPoint[i];
						pointDistances[i] = -BiharmonicDistanceP2P_sorted(weights, weightsSort, iVertexBind , m->bindings[iBind], 1.0, preValue, currentTh);
				
						maxDistance = max(pointDistances[i], maxDistance);

						outDistances << pointDistances[i] << ", ";
					}

					outDistances << endl;

					fin = clock();
					time_finalDistance = timelapse(fin,ini)*1000;
					statistics[j][k].push_back(time_finalDistance);
					statistics[j][k].push_back(maxDistance);
				}
				outDistances << endl;
			}
			distFile.close();
			// Guardamos datos
			//vector<vector<vector<float> > >statistics(interiorPoints[i].size());
			//vector<float> > time_MVC(interiorPoints[i].size(), -1);
			out << file+"_results.txt" << endl;
			out << file+"_distances.txt" << endl;
			out << file+"_pesos.txt" << endl;

			QFile resFile(file+"_results.txt");
			if(!resFile.open(QFile::WriteOnly)) return;
			QTextStream outRes(&resFile);
			outRes << "Init: " << time_init << " and AComputation: " << time_AComputation << " ms." << endl;
			outRes << time_MVC[i] << endl;
			for(int st01 = 0; st01 < time_MVC.size(); st01++)
			{
				outRes << QString("%1 ").arg(time_MVC[st01], 8);
			}
			outRes << " ms." << endl;
			for(int st01 = 0; st01 < statistics.size(); st01++)
			{
				for(int st02 = 0; st02 < statistics[st01].size(); st02++)
				{
					for(int st03 = 0; st03 < statistics[st01][st02].size(); st03++)
					{
						outRes << QString("%1, ").arg(statistics[st01][st02][st03], 8);
					}
					outRes << endl;
				}
			}
			resFile.close();

			// Exportamos la segmentacion y los pesos en un fichero.
			QFile pesosFile(file+"_pesos.txt");
			if(!pesosFile.open(QFile::WriteOnly)) return;
			QTextStream outPesos(&pesosFile);

			FILE* foutLog = fopen("C:/DATA/phd_dev/data/models_for_test/outLog.txt", "a");
			fprintf(foutLog, "%s\n", modelFiles[i][mf].toStdString().c_str()); fflush(0);
			fclose(foutLog);


			// Calculo de pesos
			for(int k = 0; k < thresholds.size(); k++)
			{
				for(int i = 0; i< m->bindings.size(); i++)
				{
					m->bindings[i]->weightsCutThreshold = thresholds[k];
				}

				printf("Computing skinning:\n");

				// Realizamos el calculo para cada binding
				clock_t iniImportante = clock();
				ComputeSkining(*m);
				clock_t finImportante = clock();

				double timeComputingSkinning = timelapse(finImportante,iniImportante);

				outPesos << "Threshold: " << thresholds[k] << " tiempo-> "<< timeComputingSkinning << endl;
				char* charIntro;
				scanf(charIntro);

				for(int bindId = 0; bindId < m->bindings.size(); bindId++)
				{
					for(int pointBind = 0; pointBind < m->bindings[bindId]->pointData.size(); pointBind++)
					{
						PointData& pt = m->bindings[bindId]->pointData[pointBind];
						outPesos << pt.ownerLabel << ", ";
					}

					outPesos << endl;
				}
			}

			pesosFile.close();

			// eliminamos el modelo
			delete m;
			escena->models.clear();
			
		}
	}
	outFile.close();
}

/////////////////////////////////////////////////
/// \brief GLWidget::computeProcess
///
void GLWidget::computeProcess()
{
	printf("Voxelization de modelos:\n");
	if(escena->models.size() <= 0) return;

	for(unsigned int i = 0; i< escena->models.size(); i++)
	{
		Modelo* m = (Modelo*)escena->models[i];
        printf("Modelo: %s\n", m->sModelPrefix.c_str());

        // Bind skeletons: Now all the skeletons
        printf("Skeleton Binding: %s\n", m->sModelPrefix.c_str());

		// Construimos tantos grafos como partes tiene el modelo
		BuildSurfaceGraphs(*m, m->bindings);

		bool usePatches = false;
		if(usePatches)
		{
			AddVirtualTriangles(*m);
		}

		// Si no se ha calculado las distancias biharmonicas lo hacemos
		// Eso es que no tiene embedding.
		//if(m->embedding.size() == 0)
		if(!m->computedBindings)
		{
			/*
			char bindingFileName[150];
			char bindingFileNameComplete[150];
			sprintf(bindingFileName, "%s/bind_%s", m->sPath.c_str(), m->sName.c_str());
			sprintf(bindingFileNameComplete, "%s.bin", bindingFileName);

			bool ascii = false;

			// A. Intentamos cargarlo
			ifstream myfile;
			myfile.open (bindingFileNameComplete, ios::in |ios::binary);
			bool loaded = false;
			if (myfile.is_open())
			{
				// En el caso de existir simplemente tenemos que cargar las distancias.
				loaded = LoadEmbeddings(*m, bindingFileNameComplete);
			}

			//B. En el caso de que no se haya cargado o haya incongruencias con lo real, lo recomputamos.
			if(!loaded) 
			{
				bool success = ComputeEmbeddingWithBD(*m);
				if(!success)
				{
					printf("[ERROR - computeProcess] No se ha conseguido computar el embedding\n");
					fflush(0);
					return;
				}
				else SaveEmbeddings(*m, bindingFileName, ascii);
			}
			*/

			bool success = ComputeEmbeddingWithBD(*m, usePatches);
		}

		//normalizeDistances(*m);

		// de momento asignamos todos los esqueletos a todos los bindings... ya veremos luego.
        for(int i = 0; i< escena->skeletons.size(); i++)
		{
			float minValue = GetMinSegmentLenght(getMinSkeletonSize((skeleton*)escena->skeletons[i]),0);
			((skeleton*)escena->skeletons[i])->minSegmentLength = minValue;

			for(int bind = 0; bind< m->bindings.size(); bind++)
				m->bindings[bind]->bindedSkeletons.push_back((skeleton*)escena->skeletons[i]);
		}

		for(int i = 0; i< m->bindings.size(); i++)
		{
			m->bindings[i]->weightsCutThreshold = escena->weightsThreshold;
		}

        printf("Computing skinning:\n");

        // Realizamos el calculo para cada binding
		 ComputeSkining(*m);
		//reportResults(*m, m->bindings[bind]);

		printf("Computing skinning:\n");
		fflush(0);
    }

	//paintModelWithData();
	//paintPlaneWithData();
}


void GLWidget::readSkeleton(string fileName)
{
	vector<skeleton*> skts;
	skts.resize(escena->skeletons.size());
	
	for(int i = 0; i< skts.size(); i++)
		skts[i] = (skeleton*)escena->skeletons[i];

	readSkeletons(fileName, skts);

     emit updateSceneView();
 }


 void GLWidget::readScene(string fileName, string name, string path)
 {
     QFile modelDefFile(fileName.c_str());
     if(modelDefFile.exists())
     {
        modelDefFile.open(QFile::ReadOnly);
        QTextStream in(&modelDefFile);

        QString sSceneName = in.readLine(); in.readLine();
        QString sGlobalPath = in.readLine(); in.readLine();
		QString sPath = in.readLine(); in.readLine(); in.readLine();
        QString sModelFile = in.readLine(); in.readLine(); in.readLine();
        QString sSkeletonFile = in.readLine(); in.readLine(); in.readLine();
        QString sEmbeddingFile = in.readLine(); in.readLine(); in.readLine();
        QString newPath(path.c_str());
        newPath = newPath +"/";
        if(!sPath.isEmpty())
            newPath = newPath+"/"+sPath +"/";

        // Leer modelo
        readModel( (newPath+sModelFile).toStdString(), sSceneName.toStdString(), newPath.toStdString());

        Modelo* m = ((Modelo*)escena->models.back());
		m->sPath = newPath.toStdString(); // importante para futuras referencias

        // Leer esqueleto
        readSkeleton((newPath+sSkeletonFile).toStdString());

        // Leer embedding
        ReadEmbedding((newPath+sEmbeddingFile).toStdString(), m->embedding);

        modelDefFile.close();
    }
 }


 void GLWidget::readModel(string fileName, string name, string path)
 {

     // Desglosamos el fichero en 3 partes:
     //    1. Path donde esta el modelo
     //    2. Prefijo del fichero
     //    3. Extension del fichero.

     QString ext = QString(fileName.c_str()).right(3);

     if(ext == QString("off") || ext == QString("obj")) // cargar modelo simple
     {
         escena->models.push_back((object*)new Modelo(escena->getNewId()));

         printf("%s\n%s\n%s\n%s\n",fileName.c_str(),name.c_str(),ext.toStdString().c_str(),path.c_str() ); fflush(0);

         ((Modelo*)escena->models.back())->loadModel(fileName, name, ext.toStdString(), path);

		 /*
         if(((Modelo*)escena->models.back())->cleanModel())
         {
             printf("Se ha guardado un nuevo modelo limpio...\n");
             ((Modelo*)escena->models.back())->saveModel(QString("%1%2_cleaned.%3").arg(path.c_str()).arg(name.c_str()).arg(ext).toStdString(), name, ext.toStdString(), path);
         }
         else
         {
             printf("El modelo est� limpio...\n");
         }
		 */

         /*
         escena->models.push_back((object*)new Modelo(escena->getNewId()));
         ((Modelo*)escena->models.back())->loadModel(fileName, name, ext.toStdString(), path);
         ((Modelo*)escena->models.back())->addTranslation(1,0,0);
         ((Modelo*)escena->models.back())->freezeTransformations();
         //((Modelo*)escena->models.back())->shading->selected = true;
         //((Modelo*)escena->models.back())->shading->subObjectmode = true;

         //for(int i = 100; i< 200; i++)
         //   ((Modelo*)escena->models.back())->shading->selectedIds.push_back(i);



         escena->models.push_back((object*)new Modelo(escena->getNewId()));
         ((Modelo*)escena->models.back())->loadModel(fileName, name, ext.toStdString(), path);
         ((Modelo*)escena->models.back())->addRotation(0,90,0);
         ((Modelo*)escena->models.back())->addTranslation(-1,0,0);
         ((Modelo*)escena->models.back())->freezeTransformations();

         escena->models.push_back((object*)new Modelo(escena->getNewId()));
         ((Modelo*)escena->models.back())->loadModel(fileName, name, ext.toStdString(), path);
         ((Modelo*)escena->models.back())->addTranslation(1.5,0,0);
         ((Modelo*)escena->models.back())->freezeTransformations();
         */

     }
     else if(ext == QString("txt")) // Cargamos una configuracion
     {
         QFile modelDefFile(fileName.c_str());
         if(modelDefFile.exists())
         {
             modelDefFile.open(QFile::ReadOnly);
             QTextStream in(&modelDefFile);

             QString sPath, modelFileName, cageFileName;
             vector< QString > cageStills;
             QString stillCage;

             // Lectura del modelo
             in >> sPath;
             in >> modelFileName;

             QFileInfo sPathAux(fileName.c_str());
             QString absPath = sPathAux.canonicalPath();

             sPath = absPath+sPath;

             escena->models.push_back((object*)new Modelo(escena->getNewId()));
             Modelo* m = (Modelo*)escena->models.back();

             QString ext2 = modelFileName.right(3);
             QString fileNameAbs = (sPath+"/"+modelFileName);
             QString nameAbs = modelFileName;
             nameAbs.chop(4);


             // Modelo
             m->loadModel(fileNameAbs.toStdString(),
                          nameAbs.toStdString(),
                          ext2.toStdString(),
                          sPath.toStdString());

             if(!in.atEnd())
             {
                 in >> cageFileName;

                 ext2 = cageFileName.right(3);
                 nameAbs = cageFileName;
                 nameAbs.chop(4);
                 fileNameAbs = (sPath+"/"+cageFileName);

                 // Caja
                 m->modelCage = new Cage(escena->getNewId());
                 m->modelCage->loadModel(fileNameAbs.toStdString(),
                                         nameAbs.toStdString(),
                                         ext2.toStdString(),
                                         sPath.toStdString());
                 m->modelCage->type = MODEL_CAGE;

                 // Caja dinamica
                 m->dynCage = new Cage(escena->getNewId());
                 m->dynCage->loadModel(fileNameAbs.toStdString(),
                                       nameAbs.append("_dyn").toStdString(),
                                       ext2.toStdString(),
                                       sPath.toStdString());
                 m->dynCage->type = DYN_CAGE;

                 Cage* cg = m->dynCage;
                 loadSelectableVertex(cg);

                 while(!in.atEnd())
                 {
                     in >> stillCage;
                     cageStills.push_back(stillCage);
                 }

                 if((int)cageStills.size() > 0 )
                 {
                     // Still cages
                     //m->stillCages.resize(cageStills.size());
                     for(unsigned int i = 0; i< cageStills.size(); i++)
                     {
                         ext2 = cageStills[i].right(3);
                         nameAbs = cageStills[i];
                         nameAbs.chop(4);
                         fileNameAbs = (sPath+"/"+cageStills[i]);

                         m->stillCages.push_back(new Cage(escena->getNewId()));

                         m->stillCages.back()->loadModel(fileNameAbs.toStdString(),
                                                         nameAbs.toStdString(),
                                                         ext2.toStdString(),
                                                         sPath.toStdString());
                         m->stillCages.back()->type = STILL_CAGE;

                         //parent->ui->cagesComboBox->addItem(cageStills[i].left(cageStills[i].length()-4), i);
                     }
                 }
             }

             modelDefFile.close();
         }
     }

     ReBuildScene();
     updateInfo();
     emit updateSceneView();
 }

 /*
// Lee una o varias cajas del disco
// filename: fichero de ruta completa.
// model: MyMesh donde se almacenara la/s cajas leida/s.
void GLWidget::readCage(QString fileName, Modelo& m_)
{
    // Desglosamos el fichero en 3 partes:
    //    1. Path donde est� el modelo
    //    2. Prefijo del fichero
    //    3. Extensi�n del fichero.
    if(!loadedModel) return; // Es necesario cargar el modelo antes.

    QString ext = fileName.right(3);
    QFileInfo sPathAux(fileName);
    //m_.sModelPath = aux.toStdString();

    bool loaded = false;
    if(ext == "off")
    {
        //fileName cage:
        m_.cage.Clear();
        m_.dynCage.Clear();

        vcg::tri::io::ImporterOFF<MyMesh>::Open(m_.cage,fileName.toStdString().c_str());
        gpUpdateNormals(m_.cage, false);
        vcg::tri::Append<MyMesh, MyMesh>::Mesh(m_.dynCage,m_.cage, false);
        gpUpdateNormals(m_.dynCage, false);

        loadedModel = loaded = true;
        updateInfo();

        viewingMode = DynCage_Mode;
    }
    else if(ext == "txt")
    {
        //fileName cage:
        m_.cage.Clear();
        m_.dynCage.Clear();

        QFile stillCagesDef(fileName);
        if(stillCagesDef.exists())
        {
            stillCagesDef.open(QFile::ReadOnly);
            QTextStream sciTS(&stillCagesDef);
            int stillCagesCount = 0;
            sciTS >> stillCagesCount;
            if(stillCagesCount > 0)
                m_.stillCages.resize(stillCagesCount);

            QString sPathModels = QString(m_.sModelPath.c_str())+"/";

            QString str;
            QString cageName;

            sciTS >> cageName >> str;
            printf("Caja original: %s\n", (sPathModels+str).toStdString().c_str()); fflush(0);
            vcg::tri::io::ImporterOFF<MyMesh>::Open(m_.cage,cageName.toStdString().c_str());
            gpUpdateNormals(m_.cage, false);

            for(unsigned int i = 0; i< m_.stillCages.size(); i++)
            {
                str = ""; cageName = "";

                sciTS >> cageName >> str;
                printf("Caja %d: %s\n", i, (sPathModels+str).toStdString().c_str()); fflush(0);

                if(!QFile((sPathModels+str)).exists())
                    continue;

                parent->ui->cagesComboBox->addItem(cageName, i);

                m_.stillCages[i] = new MyMesh();
                vcg::tri::io::ImporterOFF<MyMesh>::Open(*m_.stillCages[i],(sPathModels+str).toStdString().c_str());
                gpUpdateNormals(*m_.stillCages[i], false);
            }
            stillCagesDef.close();

            viewingMode = Cages_Mode;
        }

        loadedCages = loaded = true;
        updateInfo();

    }
    else
        return;


    if(loaded)
    {
        vcg::tri::UpdateBounding<MyMesh>::Box(m_.modeloOriginal);
        vcg::tri::UpdateBounding<MyMesh>::Box(m_.cage);

        //buildCube(cage, model.bbox.Center(), model.bbox.Diag());
        //loadSelectVertexCombo(cage);
        loadSelectableVertex(m_.dynCage);

        // Podriamos poner un switch para leer varios tipos de modelos.

        ReBuildScene();
    }

}
*/

void GLWidget::updateColorLayersWithSegmentation(int maxIdx)
{
    //TODO
    assert(false);
    /*
    vertexColorLayers.resize(2);
    int segLayer = 0;
    int spotLayer = 1;

    vertexColorLayers[segLayer].resize(m.modeloOriginal.vn);
    vertexColorLayers[spotLayer].resize(m.modeloOriginal.vn);

    MyMesh::VertexIterator vii;
    for(vii = m.modeloOriginal.vert.begin(); vii!=m.modeloOriginal.vert.end(); ++vii )
    {
        //vector< int > idxCounters(maxIdx+1, 0);

        int different = 0;
        int neighbours = 0;
        int idxOfVii = BHD_indices[vii->IMark()];
        vcg::face::VFIterator<MyFace> vfi(&(*vii)); //initialize the iterator to the first face
        for(;!vfi.End();++vfi)
        {
          MyFace* f = vfi.F();

          for(int i = 0; i<3; i++)
          {
            MyVertex * v = f->V(i);
            if (v->IMark() == vii->IMark())continue;

            if( BHD_indices[v->IMark()] != idxOfVii )
                different++;

            neighbours++;
          }
       }

        if(different > neighbours/2.0)
            vertexColorLayers[spotLayer][vii->IMark()] = QColor(255,0,0);
        else
            vertexColorLayers[spotLayer][vii->IMark()] = QColor(255,255,255);

        vertexColorLayers[segLayer][vii->IMark()] = GetColour(BHD_indices[vii->IMark()], 0, maxIdx);
    }

    colorLayers = true;
    colorsLoaded = true;
    viewingMode = BHD_Mode;

    updateGL()
    */
}

void GLWidget::importSegmentation(QString fileName)
{
    //TODO

    /*
    // Leemos contenido del paquete
    QFile file(fileName);
    file.open(QFile::ReadOnly);
    QTextStream in(&file);

    QString indicesFileName;
    indicesFileName = in.readLine();

    int maxIdx = -1;

    QString dummy;
    int number_of_points;
    in >> number_of_points;

    //printf("%d\n", number_of_points); fflush(0);

    if(number_of_points <= 0)
        return;

    BHD_indices.resize(m.modeloOriginal.vn);
    int counter = 0;
    for(int i = 0; i< m.modeloOriginal.vn && !in.atEnd(); i++)
    {
        in >> dummy;
        //printf("%s ", dummy.toStdString().c_str()); fflush(0);

        for(int j = 0; j< number_of_points && !in.atEnd(); j++)
        {
            int tempData;
            in >> tempData;

            //printf("%d ", tempData); fflush(0);
            if(j == 0)
            {
                BHD_indices[i] = tempData; // Cogemos el primero
            }

            maxIdx = max(maxIdx, BHD_indices[i]);
            counter++;
        }
        //printf("\n"); fflush(0);
    }

    if(counter != m.modeloOriginal.vn)
        printf("No coincide el numero de vertices con colores.\n");

    if(maxIdx <= 0) return;

    printf("Distancias-> %d elementos\n", maxIdx); fflush(0);

    updateColorLayersWithSegmentation(maxIdx);
    */
}

void GLWidget::readDistances(QString fileName)
{
    // Leemos contenido del paquete
    int indSize = 0, vertSize = 0;
    QFile file(fileName);
    file.open(QFile::ReadOnly);
    QTextStream in(&file);
    QString indicesFileName, distancesFileName;
    indicesFileName = in.readLine();
    distancesFileName = in.readLine();
    in >> vertSize; // Leemos el numero de vert del modelo
    in >> indSize; // Leemos el numero de indices calculados.


    file.close();

    if(vertSize == 0 || indSize == 0) return;

    int i = 0;
    // Leemos indices
    if(indSize == vertSize)
    {
        BHD_indices.resize(indSize);
        for(unsigned int i = 0; i< BHD_indices.size(); i++)
            BHD_indices[i] = i;
    }
    else
    {
        QFile indFile(indicesFileName);
        indFile.open(QFile::ReadOnly);
        QTextStream inIndices(&indFile);
        BHD_indices.resize(indSize);

        int dummy = 0;
        while(!inIndices.atEnd())
        {
           inIndices >> dummy;
           BHD_indices[i] = dummy;
           i++;
        }
        indFile.close();
    }

    // Leemos las distancias
    QFile distFile(distancesFileName);
    distFile.open(QFile::ReadOnly);
    QTextStream inDistances(&distFile);

    /* LECTURA TOTAL */
    /*
    i = 0;
    int fila = 0, cola = 0;
    while(!inDistances.atEnd())
    {
       if(i%indSize == 0)
       {
         BHD_distancias[fila].resize(indSize);
         fila++;
       }

       inIndices >> dummy;
       BHD_distancias[fila][cola] = dummy;
       i++;
       cola++;
    }
    distFile.close();
    */

    BHD_distancias.resize(vertSize);
    for(int k = 0; k < indSize; k++)
    {
        BHD_distancias[k].resize(indSize);
        BHD_distancias[k][k] = 0;
    }

    /* LECTURA TIPO MATLAB */
    i = 0;
    QString dummy = "0";
    int fila = 0, cola = 0;
    while(!inDistances.atEnd())
    {
        i++;
        cola++;

        if(i%indSize == 0)
        {
          fila++;
          cola = fila+1;
          i += fila+1;
        }

        inDistances >> dummy;
        if(fila < indSize && cola < indSize)
        {
            BHD_distancias[fila][cola] = dummy.toDouble();
            BHD_distancias[cola][fila] = dummy.toDouble();
        }

    }
    distFile.close();

    viewingMode = BHD_Mode;
}

// This function inits the scene using the general viewing parameters.
 void GLWidget::initScene(){
     setGridIsDrawn(false);
     setAxisIsDrawn(true);
     setFPSIsDisplayed(true);

     // Restore previous viewer state.
     restoreStateFromFile();
 }

 // Inicializates the scene taking in account all the models of the scene.
 void GLWidget::ReBuildScene(){

     double minX = 0, minY = 0, minZ = 0, maxX = 0, maxY = 0, maxZ = 0;
     bool init_ = false;
     // Recomponemos la bounding box de la escena
     for(unsigned int i = 0; i< escena->models.size(); i++)
     {
         Point3d minAuxPt, maxAuxPt;
         escena->models[i]->getBoundingBox(minAuxPt, maxAuxPt);


         if(!init_)
         {
             minX = minAuxPt.X();
             minY = minAuxPt.Y();
             minZ = minAuxPt.Z();

             maxX = maxAuxPt.X();
             maxY = maxAuxPt.Y();
             maxZ = maxAuxPt.Z();
             init_  = true;
         }
         else
         {
             minX = min(minAuxPt.X(), minX);
             minY = min(minAuxPt.Y(), minY);
             minZ = min(minAuxPt.Y(), minY);

             maxX = max(maxAuxPt.X(), maxX);
             maxY = max(maxAuxPt.Y(), maxY);
             maxZ = max(maxAuxPt.Z(), maxZ);
         }
     }

     for(unsigned int i = 0; i< escena->skeletons.size(); i++)
     {
         // TODO
         // Queda mirar esto... lo que tengo previsto es pedir al esqueleto... como con el modelo.

         Point3d minAuxPt, maxAuxPt;
         escena->skeletons[i]->getBoundingBox(minAuxPt, maxAuxPt);

         if(!init_)
         {
             minX = minAuxPt.X();
             minY = minAuxPt.Y();
             minZ = minAuxPt.Z();

             maxX = maxAuxPt.X();
             maxY = maxAuxPt.Y();
             maxZ = maxAuxPt.Z();
             init_ = true;
         }
         else
         {
             minX = min(minAuxPt.X(), minX);
             minY = min(minAuxPt.Y(), minY);
             minZ = min(minAuxPt.Y(), minY);

             maxX = max(maxAuxPt.X(), maxX);
             maxY = max(maxAuxPt.Y(), maxY);
             maxZ = max(maxAuxPt.Z(), maxZ);
         }
     }

     Point3d minPt(minX,minY,minZ);
     Point3d maxPt(maxX,maxY,maxZ);

     for(int i = 0; i< 3; i++) m_sceneMin[i] = (float)minPt[i] ;
     for(int i = 0; i< 3; i++) m_sceneMax[i] = (float)maxPt[i] ;
     m_ptCenter = Point3d((minX+maxX)/2, (minY+maxY)/2, (minZ+maxZ)/2);

    // definimos las condiciones de la escena para el GlViewer.
    setSceneBoundingBox(Vec(m_sceneMin[0],m_sceneMin[1],m_sceneMin[2]),Vec(m_sceneMax[0],m_sceneMax[1],m_sceneMax[2]));

    Point3d minPoint(m_sceneMin[0],m_sceneMin[1],m_sceneMin[2]);

    setSceneCenter(Vec(m_ptCenter.X(),m_ptCenter.Y(),m_ptCenter.Z()));
    setSceneRadius((m_ptCenter - minPoint).Norm());

    printf("SceneMinMax: (%f,%f,%f)-(%f,%f,%f)\n",m_sceneMin[0],m_sceneMin[1],m_sceneMin[2],m_sceneMax[0],m_sceneMax[1],m_sceneMax[2]);
    printf("SceneCenter: (%f,%f,%f)\n",m_ptCenter.X(),m_ptCenter.Y(),m_ptCenter.Z());

    showEntireScene();

    // Actualizamos la vista.
    updateGL();

    // Guardamos la vista.
    saveStateToFile();

    //firstInit = false;
}

 void GLWidget::drawWithNames()
 {
     //TODO
     /*

     //MyMesh *currentModelo;
     MyMesh *currentCage;

     currentCage = &m.dynCage;


     //if(!showDeformedModel)
     //    currentModelo = &m.modeloOriginal;
     //else
     //{
     //    if(activeCoords == HARMONIC_COORDS)
     //        currentModelo = &m.newModeloHC;
     //    else
     //        currentModelo = &m.newModeloGC;
     //}


     if(drawCage)
     {
         glPointSize(5);
         glColor3f(0.1, 1.0, 0.1);
         glDisable(GL_LIGHTING);
         MyMesh::VertexIterator vi;
         for(vi = currentCage->vert.begin(); vi!=currentCage->vert.end(); ++vi )
         {
             int id = vi->IMark();
             glPushName(id);
             glBegin(GL_POINTS);
             glVertex3dv(&(*vi).P()[0]);
             glEnd();
             glPopName();
         }
         glEnd();
         glEnable(GL_LIGHTING);
     }
     */
 }

 /*
 void GLWidget::drawCube(Point3d o, double cellSize, Point3f color, bool blend)
 {
     Point3f v[8];

     v[0] = Point3f(o.X(), o.Y()+cellSize, o.Z());
     v[1] = Point3f(o.X(), o.Y()+cellSize, o.Z()+cellSize);
     v[2] = Point3f(o.X(), o.Y(), o.Z()+cellSize);
     v[3] = Point3f(o.X(), o.Y(), o.Z());
     v[4] = Point3f(o.X()+cellSize, o.Y()+cellSize, o.Z());
     v[5] = Point3f(o.X()+cellSize, o.Y()+cellSize, o.Z()+cellSize);
     v[6] = Point3f(o.X()+cellSize, o.Y(), o.Z()+cellSize);
     v[7] = Point3f(o.X()+cellSize, o.Y(), o.Z());

     glDisable(GL_LIGHTING);

     if(blend)
         glColor4f(color[0], color[1], color[2], 0.3);
     else
         glColor3fv(&color[0]);

     glBegin(GL_QUADS);
     // queda reconstruir el cubo y ver si se pinta bien y se ha calculado correctamente.
        glNormal3f(-1,0,0);
        glVertex3fv(&v[0][0]); glVertex3fv(&v[1][0]); glVertex3fv(&v[2][0]); glVertex3fv(&v[3][0]);

        glNormal3f(1,0,0);
        glVertex3fv(&v[7][0]); glVertex3fv(&v[6][0]); glVertex3fv(&v[5][0]); glVertex3fv(&v[4][0]);

        glNormal3f(0,1,0);
        glVertex3fv(&v[1][0]); glVertex3fv(&v[5][0]); glVertex3fv(&v[6][0]); glVertex3fv(&v[2][0]);

        glNormal3f(0,-1,0);
        glVertex3fv(&v[0][0]); glVertex3fv(&v[3][0]); glVertex3fv(&v[7][0]); glVertex3fv(&v[4][0]);

        glNormal3f(0,0,1);
        glVertex3fv(&v[6][0]); glVertex3fv(&v[7][0]); glVertex3fv(&v[3][0]); glVertex3fv(&v[2][0]);

        glNormal3f(0,0,-1);
        glVertex3fv(&v[0][0]); glVertex3fv(&v[4][0]); glVertex3fv(&v[5][0]); glVertex3fv(&v[1][0]);
     glEnd();
     glEnable(GL_LIGHTING);
 }
*/

 void GLWidget::drawModel()
 {

 }

 void GLWidget::drawWithDistances()
 {
     //TODO
     /*
     if(ShadingModeFlag)
        glShadeModel(GL_SMOOTH);
     else
        glShadeModel(GL_FLAT);

     bool useColorLayers = (colorLayers && colorLayerIdx >= 0 && colorLayerIdx < vertexColorLayers.size());
     fflush(0);
     if(loadedModel)
     {
         // Pintamos en modo directo el modelo
         glBegin(GL_TRIANGLES);
         glColor3f(1.0, 1.0, 1.0);
         MyMesh::FaceIterator fi;
         int faceIdx = 0;
         for(fi = m.modeloOriginal.face.begin(); fi!=m.modeloOriginal.face.end(); ++fi )
          {
              //glNormal3dv(&(*fi).N()[0]);
              for(int i = 0; i<3; i++)
              {
                  int id = fi->V(i)->IMark();
                  if((colorsLoaded&&!colorLayers) || useColorLayers)
                  {
                      if(useColorLayers)
                      {
                         glColor3f(vertexColorLayers[colorLayerIdx][id].redF(),vertexColorLayers[colorLayerIdx][id].greenF(), vertexColorLayers[colorLayerIdx][id].blueF());
                      }
                      else
                          glColor3f(vertexColors[id].redF(),vertexColors[id].greenF(), vertexColors[id].blueF());

                  }
                  else
                    glColor3f(1.0,1.0,1.0);

                  glNormal3dv(&(*fi).V(i)->N()[0]);
                  glVertex3dv(&(*fi).V(i)->P()[0]);
              }
              faceIdx++;
          }
         glEnd();

     }
             */
 }

 void GLWidget::drawWithCages()
 {
     //TODO
     /*

     MyMesh *currentModelo;
     MyMesh *currentCage;


     //if(!showDeformedModel)
     //{
     //    currentModelo = &modelo;
     //    currentCage = &cage;
     //}
     //else
     //{
     //    currentModelo = &newModelo;
     //    currentCage = &newCage;
     //}


     if(stillCageAbled && stillCageSelected >= 0)
         currentCage = m.stillCages[stillCageSelected];
     else
         currentCage = &m.dynCage;

     if(!showDeformedModel)
         currentModelo = &m.modeloOriginal;
     else
     {
         if(activeCoords == HARMONIC_COORDS)
             currentModelo = &m.newModeloHC;
         else if(activeCoords == GREEN_COORDS)
             currentModelo = &m.newModeloGC;
         else
             currentModelo = &m.newModeloMVC;
     }


      //MyMesh::PerVertexAttributeHandle<unsigned int > vertIdxHnd = vcg::tri::Allocator<MyMesh>::GetPerVertexAttribute<unsigned int >(*currentModelo,"vertIdx");

     glShadeModel(GL_SMOOTH);
     if(loadedModel)
     {
         //vector<float> normalsArray;
         //vector<float> vertexes;
         //vector<float> colors;
         */
         /*
         MyMesh::FaceIterator fi;
         int faceIdx = 0;
         for(fi = currentModelo->face.begin(); fi!=currentModelo->face.end(); ++fi )
          {
              //glNormal3dv(&(*fi).N()[0]);
              for(int i = 0; i<3; i++)
              {
                  if(shadeCoordInfluence || m_bDrawHCInfluences)
                  {
                      QColor color;
                      if(activeCoords == GREEN_COORDS && bGCcomputed)
                        color = GetColour(selectionValuesForColorGC[fi->V(i)->IMark()], maxMinGC[1],maxMinGC[0]);

                      else if(activeCoords == HARMONIC_COORDS && bHComputed)
                          color = GetColour(selectionValuesForColorHC[fi->V(i)->IMark()], maxMinHC[1],maxMinHC[0]);

                      else if(activeCoords == MEANVALUE_COORDS && bMVCComputed)
                          color = GetColour(selectionValuesForColorMVC[fi->V(i)->IMark()], maxMinMVC[1],maxMinMVC[0]);

                      else color = QColor(255,255,255);

                      colors.push_back(color.redF());
                      colors.push_back(color.greenF());
                      colors.push_back(color.blueF());

                  }
                  else
                  {
                      colors.push_back(1.0);
                      colors.push_back(1.0);
                      colors.push_back(1.0);
                  }

                  normalsArray.push_back((*fi).V(i)->N()[0]);
                  normalsArray.push_back((*fi).V(i)->N()[1]);
                  normalsArray.push_back((*fi).V(i)->N()[2]);

                  vertexes.push_back((*fi).V(i)->P()[0]);
                  vertexes.push_back((*fi).V(i)->P()[1]);
                  vertexes.push_back((*fi).V(i)->P()[2]);
              }
              faceIdx++;
          }


         //glBegin(GL_TRIANGLES);



         //glEnableClientState(GL_NORMAL_ARRAY);
         //glEnableClientState(GL_COLOR_ARRAY);
         glEnableClientState(GL_VERTEX_ARRAY);

         GLuint buffers[3];

         glGenBuffers(3, buffers);

           glBindBuffer(GL_ARRAY_BUFFER, buffers[0]);
           glBufferData(GL_ARRAY_BUFFER, sizeof(float)*vertexes.size(), &vertexes[0], GL_STATIC_DRAW);
           glVertexPointer(3, GL_FLOAT, sizeof(float)*3, BUFFER_OFFSET(0));

           //glBindBuffer(GL_ARRAY_BUFFER, buffers[1]);
           //glBufferData(GL_ARRAY_BUFFER, sizeof(float)*normalsArray.size(), &normalsArray[0], GL_STATIC_DRAW);
           //glVertexPointer(3, GL_FLOAT, sizeof(float)*3, BUFFER_OFFSET(0));

           //glBindBuffer(GL_ARRAY_BUFFER, buffers[2]);
           //glBufferData(GL_ARRAY_BUFFER, sizeof(float)*colors.size(), &colors[0], GL_STATIC_DRAW);
           //glVertexPointer(3, GL_FLOAT, sizeof(float)*3, BUFFER_OFFSET(0));


         glDrawArrays(GL_TRIANGLES, 0, vertexes.size()*sizeof(float));

         // deactivate vertex arrays after drawing
         glDisableClientState(GL_VERTEX_ARRAY);
         //glDisableClientState(GL_COLOR_ARRAY);
         //glDisableClientState(GL_NORMAL_ARRAY);

         glDeleteBuffers(3, &buffers[0]);
         */
         /*
         glColor3f(1.0, 1.0, 1.0);
         glColor3f(color.redF(),color.greenF(), color.());
         glNormal3dv(&(*fi).V(i)->N()[0]);
         glVertex3dv(&(*fi).V(i)->P()[0]);
         */
         //glEnd();

         /*
         // Pintamos en modo directo el modelo
         glBegin(GL_TRIANGLES);
         glColor3f(1.0, 1.0, 1.0);
         MyMesh::FaceIterator fi;
         int faceIdx = 0;
         for(fi = currentModelo->face.begin(); fi!=currentModelo->face.end(); ++fi )
          {
              //glNormal3dv(&(*fi).N()[0]);
              for(int i = 0; i<3; i++)
              {
                  if(shadeCoordInfluence || m_bDrawHCInfluences)
                  {
                      QColor color;
                      if(activeCoords == GREEN_COORDS && bGCcomputed)
                      {
                          color = GetColour(selectionValuesForColorGC[fi->V(i)->IMark()], maxMinGC[1],maxMinGC[0]);
                          //GetColour(selectionValuesForColorGC[fi->V(i)->IMark()], 0, 1);
                      }


                      else if(activeCoords == HARMONIC_COORDS && bHComputed)
                      {
                          //GetColour(selectionValuesForColorHC[fi->V(i)->IMark()], 0, 1);
                          color = GetColour(selectionValuesForColorHC[fi->V(i)->IMark()], maxMinHC[1],maxMinHC[0]);
                      }

                      else if(activeCoords == MEANVALUE_COORDS && bMVCComputed)
                      {
                          //GetColour(selectionValuesForColorMVC[fi->V(i)->IMark()], -1, 1);
                          color = GetColour(selectionValuesForColorMVC[fi->V(i)->IMark()], maxMinMVC[1],maxMinMVC[0]);
                      }

                      else color = QColor(255,255,255);

                      glColor3f(color.redF(),color.greenF(), color.blueF());
                  }
                  else
                      glColor3f(1.0, 1.0, 1.0);

                  glNormal3dv(&(*fi).V(i)->N()[0]);
                  glVertex3dv(&(*fi).V(i)->P()[0]);
              }
              faceIdx++;
          }
         glEnd();



         if(drawCage)
         {
             glDisable(GL_LIGHTING);
             glBegin(GL_LINES);
             glColor3f(0.1, 0.1, 1.0);
             MyMesh::FaceIterator fi;
             for(fi = currentCage->face.begin(); fi!=currentCage->face.end(); ++fi )
                  {
                      for(int i = 0; i<3; i++)
                      {
                          glVertex3dv(&(*fi).P(i)[0]);
                          glVertex3dv(&(*fi).P((i+1)%3)[0]);
                      }
                  }
             glEnd();


             glPointSize(5);
             glColor3f(0.1, 1.0, 0.1);

             int idx = 0;
             MyMesh::VertexIterator vi;
             for(vi = currentCage->vert.begin(); vi!=currentCage->vert.end(); ++vi )
             {
                 glBegin(GL_POINTS);
                 if(shadeCoordInfluence && selection_.contains(idx))
                 {
                     glEnd();
                     glPointSize(12);
                     glColor3f(1.0, 0.1, 0.1);
                     glBegin(GL_POINTS);
                     glVertex3dv(&(*vi).P()[0]);
                     glEnd();
                     glPointSize(5);
                     glColor3f(0.1, 1.0, 0.1);
                     glBegin(GL_POINTS);
                 }
                 else
                     glVertex3dv(&(*vi).P()[0]);

                 idx++;
                 glEnd();
             }
            glEnable(GL_LIGHTING);
         }

         if(m_bShowHCGrid)
         {
             Point3f greenCube(0.5,1.0,0.5);
             Point3f redCube(1.0,0.5,0.5);
             Point3f blueCube(0.5,0.5,1.0);

             if(m_bShowAllGrid)
             {
                 for(int i = 0; i< m.HCgrid.dimensions.X(); i++)
                 {
                     for(int j = 0; j< m.HCgrid.dimensions.Y(); j++)
                     {
                         for(int k = 0; k< m.HCgrid.dimensions.Z(); k++)
                         {
                             Point3d o(m.HCgrid.bounding.min + Point3d(i,j,k)*m.HCgrid.cellSize);

                             if(m_bShowHCGrid_interior && m.HCgrid.cells[i][j][k]->tipo == INTERIOR)
                                 drawCube(o, m.HCgrid.cellSize, greenCube);

                             if(m_bShowHCGrid_exterior && m.HCgrid.cells[i][j][k]->tipo == EXTERIOR)
                                 drawCube(o, m.HCgrid.cellSize, blueCube);

                             if(m_bShowHCGrid_boundary && m.HCgrid.cells[i][j][k]->tipo == BOUNDARY)
                             {
                                 drawCube(o, m.HCgrid.cellSize, redCube);
                             }
                         }
                     }
                 }
             }
             else
             {
                 int j = currentDrawingSlice;
                 for( int i = 0; i< m.HCgrid.dimensions.X(); i++)
                 {
                     for( int k = 0; k< m.HCgrid.dimensions.Z(); k++)
                     {
                         Point3d o(m.HCgrid.bounding.min + Point3d(i,j,k)*m.HCgrid.cellSize);

                         if(m_bDrawHCInfluences)
                         {
                             glEnable(GL_BLEND);
                             if((unsigned int) i >= m.HCgrid.cells.size())
                                 continue;

                             if(j < 0 || j >= (int)m.HCgrid.cells[i].size())
                                 continue;

                             if((unsigned int) k >= m.HCgrid.cells[i][j].size())
                                 continue;

                             QColor c;
                             c = GetColour(sliceValues[i][k],maxMinHC[1],maxMinHC[0]);


                             Point3f color(c.redF(),c.greenF(), c.blueF());

                             if(m_bShowHCGrid_interior && m.HCgrid.cells[i][j][k]->tipo == INTERIOR)
                                 drawCube(o, m.HCgrid.cellSize, color, m_bDrawHCInfluences);

                             if(m_bShowHCGrid_exterior && m.HCgrid.cells[i][j][k]->tipo == EXTERIOR)
                                 drawCube(o, m.HCgrid.cellSize, color, m_bDrawHCInfluences);

                             if(m_bShowHCGrid_boundary && m.HCgrid.cells[i][j][k]->tipo == BOUNDARY)
                             {
                                 drawCube(o, m.HCgrid.cellSize, color, m_bDrawHCInfluences);
                             }
                             glDisable(GL_BLEND);
                         }
                         else
                         {
                             if(m_bShowHCGrid_interior && m.HCgrid.cells[i][j][k]->tipo == INTERIOR)
                                 drawCube(o, m.HCgrid.cellSize, greenCube);

                             if(m_bShowHCGrid_exterior && m.HCgrid.cells[i][j][k]->tipo == EXTERIOR)
                                 drawCube(o, m.HCgrid.cellSize, blueCube);

                             if(m_bShowHCGrid_boundary && m.HCgrid.cells[i][j][k]->tipo == BOUNDARY)
                             {
                                 drawCube(o, m.HCgrid.cellSize, redCube);
                             }
                         }
                     }
                 }
             }
         }
     }

     // Draws manipulatedFrame (the set's rotation center)
     if (selection_.size()>0)
       {
         glPushMatrix();
         glMultMatrixd(manipulatedFrame()->matrix());
         drawAxis(0.5);
         glPopMatrix();
       }

     // Draws rectangular selection area. Could be done in postDraw() instead.
     if (selectionMode_ != NONE)
       drawSelectionRectangle();

       */
 }

 // limpiamos variables de shading sobre los seleccionados
 // (ahora sobre todos)
 void GLWidget::cleanShadingVariables()
 {
     for(unsigned int i = 0; i< escena->models.size(); i++)
     {
         if(!escena->models[i])
             continue;

         escena->models[i]->shading->shtype = T_POLY;
     }
 }

 // activamos el modo lineas.
 void GLWidget::toogleModelToLines()
 {
     for(unsigned int i = 0; i< escena->models.size(); i++)
     {
         if(!escena->models[i])
             continue;

         escena->models[i]->shading->shtype = T_LINES;
     }
 }

 void GLWidget::updateGridVisualization()
 {
	 for(unsigned int i = 0; i< escena->visualizers.size(); i++)
	 {
		 if(!escena->visualizers[i] || escena->visualizers[i]->iam != GRIDRENDERER_NODE)
			 continue;

		 paintGrid((gridRenderer*)escena->visualizers[i]);
		 paintModelWithGrid();
		 ((gridRenderer*)escena->visualizers[i])->propagateDirtyness();
	 }
 }

 void GLWidget::changeVisualizationMode(int mode)
 {
	 escena->iVisMode = mode;
	 paintModelWithData();

     for(unsigned int i = 0; i< escena->visualizers.size(); i++)
     {
         if(!escena->visualizers[i] || escena->visualizers[i]->iam != GRIDRENDERER_NODE)
             continue;

          ((gridRenderer*)escena->visualizers[i])->iVisMode = mode;
          paintGrid((gridRenderer*)escena->visualizers[i]);
		  paintModelWithGrid();
		  ((gridRenderer*)escena->visualizers[i])->propagateDirtyness();
     }
 }

 // activamos el modo invisible.
 void GLWidget::toogleToShowSegmentation(bool toogle)
 {
     for(unsigned int i = 0; i< escena->visualizers.size(); i++)
     {
         if(!escena->visualizers[i] || escena->visualizers[i]->iam != GRIDRENDERER_NODE)
             continue;

          ((gridRenderer*)escena->visualizers[i])->bshowSegmentation = toogle;
          ((gridRenderer*)escena->visualizers[i])->propagateDirtyness();
     }
 }


void GLWidget::cleanWeights(gridRenderer* grRend)
{
	for(unsigned int i = 1; i < grRend->grid->cells.size()-1; i++)
	{
		for(unsigned int j = 1; j < grRend->grid->cells[i].size()-1; j++)
		{
			for(unsigned int k = 1; k < grRend->grid->cells[i][j].size()-1; k++)
			{
				if(grRend->grid->cells[i][j][k]->getType() != BOUNDARY) continue;

				cell3d* cell = grRend->grid->cells[i][j][k];
				cell->data->influences.clear();
			}
		}
	}
}

void GLWidget::changeSmoothPropagationDistanceRatio(float smoothRatioValue)
{
	smoothRatioValue = (float)0.15;
	if(DEBUG) printf("changeSmoothPropagationDistanceRatio %f\n", smoothRatioValue); fflush(0);
	
	// TODEBUG
    // Estamos actuando sobre todos los grid renderer, deberia ser solo por el activo.
    for(unsigned int i = 0; i< escena->visualizers.size(); i++)
    {
        if(!escena->visualizers[i] || escena->visualizers[i]->iam != GRIDRENDERER_NODE)
            continue;

         ((gridRenderer*)escena->visualizers[i])->grid->smoothPropagationRatio = smoothRatioValue;
         ((gridRenderer*)escena->visualizers[i])->propagateDirtyness();
    }

    //TODEBUG
    printf("Ojo!, ahora estamos teniendo en cuenta: 1 solo modelo, esqueleto y grid.\n"); fflush(0);
    gridRenderer* grRend = (gridRenderer*)escena->visualizers.back();

    if(grRend == NULL) return;

	// limpiar los pesos calculados anteriormente
	cleanWeights(grRend);

	// recalcular los pesos
    assert(false);
    // He cambiado el grid por una malla: TODO y poner la siguiente instruccion
    //computeHierarchicalSkinning(*(grRend->grid));

    grRend->propagateDirtyness();
    updateGridRender();
}


void GLWidget::changeExpansionFromSelectedJoint(float expValue)
{
     for(unsigned int i = 0; i< selMgr.selection.size(); i++)
     {
         if(((object*)selMgr.selection[i])->iam == JOINT_NODE)
         {
             joint* jt = (joint*)selMgr.selection[i];
             jt->expansion = expValue;

             for(unsigned int i = 0; i< escena->visualizers.size(); i++)
             {
                 if(!escena->visualizers[i] || escena->visualizers[i]->iam != GRIDRENDERER_NODE)
                     continue;

                  vector<DefNode>& nodes = ((gridRenderer*)escena->visualizers[i])->grid->v.intPoints;

                  for(unsigned int n = 0; n< nodes.size(); n++)
                  {
                      if(((DefNode)nodes[n]).boneId == (int)jt->nodeId)
                      {
                          if(((DefNode)nodes[n]).ratio < ratioExpansion_DEF)
                          {
                              float ratio2 = (((DefNode)nodes[n]).ratio/ratioExpansion_DEF);
                              float dif = 1-expValue;
                              float newValue =  expValue + dif*ratio2;
                              nodes[n].expansion = newValue;
                          }
                      }
                  }
             }

         }
     }

     //TODEBUG
     printf("Ojo!, ahora estamos teniendo en cuenta: 1 solo modelo, esqueleto y grid.\n"); fflush(0);
     gridRenderer* grRend = (gridRenderer*)escena->visualizers.back();

     if(grRend == NULL) return;

     //TODO: he cambiado el grid por la maya directamente, hay que rehacer la siguiente llamada.
     assert(false);
     //updateSkinningWithHierarchy(*(grRend->grid));

     grRend->propagateDirtyness();
     updateGridRender();
 }

 // activamos el modo invisible.
 void GLWidget::toogleVisibility(bool toogle)
 {
     for(unsigned int i = 0; i< escena->models.size(); i++)
     {
         if(!escena->models[i] || !escena->models[i]->shading)
             continue;

          escena->models[i]->shading->visible = !escena->models[i]->shading->visible;
     }
 }


 // activamos el modo xray.
 void GLWidget::toogleModelToXRay()
 {
     for(unsigned int i = 0; i< escena->models.size(); i++)
     {
         if(!escena->models[i])
             continue;

         escena->models[i]->shading->shtype = T_XRAY;
     }
 }

 // This function draw the elements using the state parameters.
 void GLWidget::draw()
 {
     if(ShadingModeFlag == SH_MODE_SMOOTH)
        glShadeModel(GL_SMOOTH);
     else if(ShadingModeFlag == SH_MODE_FLAT)
        glShadeModel(GL_FLAT);

     /*
     for(unsigned int i = 0; i< escena->shaders.size(); i++)
     {
         if(!escena->shaders[i] || !escena->shaders[i]->visible)
             continue;

         escena->shaders[i]->drawFunc();
     }
     */
     for(unsigned int i = 0; i< escena->visualizers.size(); i++)
     {
         if(!escena->visualizers[i] || !escena->visualizers[i]->visible)
             continue;

         ((gridRenderer*)escena->visualizers[i])->drawFunc(NULL);
     }


     for(unsigned int i = 0; i< escena->skeletons.size(); i++)
     {
         if(!escena->skeletons[i] || !escena->skeletons[i]->shading->visible)
             continue;

         ((skeleton*)escena->skeletons[i])->drawFunc();
     }


     for(unsigned int i = 0; i< escena->models.size(); i++)
     {
         if(!escena->models[i] || !escena->models[i]->shading->visible)
             continue;

         ((Modelo*)escena->models[i])->drawFunc();
     }

	 glDisable(GL_LIGHTING);
	 drawPointLocator(interiorPoint, 1, true);
	 glEnable(GL_LIGHTING);

     // Pintamos el handle en el caso de que haya selecci�n.
     selMgr.drawFunc();


    // Pintamos el handle en el caso de que haya selecci�n.
     if (selMgr.ctx != CTX_SELECTION)
       drawSelectionRectangle();
     else
        selMgr.drawFunc();

 }

 void GLWidget::postSelection(const QPoint&)
 {
	 int i = 0;
     //TODO
     /*
     if(!bHComputed && !bGCcomputed && !bMVCComputed )
         return;

     maxMinGC = Point2f(-99999, 99999);
     maxMinHC = Point2f(-99999, 99999);
     maxMinMVC = Point2f(-99999, 99999);

     if(bGCcomputed)
         for(unsigned int i = 0; i< selectionValuesForColorGC.size(); i++)
             selectionValuesForColorGC[i] = 0;

     if(bHComputed)
        for(unsigned int i = 0; i< selectionValuesForColorHC.size(); i++)
             selectionValuesForColorHC[i] = 0;

     if(bMVCComputed)
        for(unsigned int i = 0; i< selectionValuesForColorMVC.size(); i++)
             selectionValuesForColorMVC[i] = 0;

     MyMesh::VertexIterator vi;
     for(vi = m.modeloOriginal.vert.begin(); vi!=m.modeloOriginal.vert.end(); ++vi )
     {
         float valueSum = 0;
         float valueSum2 = 0;
         float valueSum3 = 0;
         for (QList<int>::const_iterator it=selection_.begin(), end=selection_.end(); it != end; ++it)
         {
             if(bGCcomputed)
                 valueSum += m.PerVertGC[vi->IMark()][*it];

             if(bHComputed)
                 valueSum2 += m.PerVertHC[vi->IMark()][*it];

             if(bMVCComputed)
                 valueSum3 += m.PerVertMVC[vi->IMark()][*it];
         }

         if(bGCcomputed)
         {
             selectionValuesForColorGC[vi->IMark()] = valueSum;
             maxMinGC[0] = max(maxMinGC[0], valueSum);
             maxMinGC[1] = min(maxMinGC[1], valueSum);
         }

         if(bHComputed)
         {
             selectionValuesForColorHC[vi->IMark()] = valueSum2;
             maxMinHC[0] = max(maxMinHC[0], valueSum2);
             maxMinHC[1] = min(maxMinHC[1], valueSum2);
         }

         if(bMVCComputed)
         {
             selectionValuesForColorMVC[vi->IMark()] = valueSum3;
             maxMinMVC[0] = max(maxMinMVC[0], valueSum3);
             maxMinMVC[1] = min(maxMinMVC[1], valueSum3);

         }
     }

     printf("\nMax GC: %f Min GC: %f \n", maxMinGC[0], maxMinGC[1]);
     printf("\nMax HC: %f Min HCC: %f \n", maxMinHC[0], maxMinHC[1]);
     printf("\nMax MVC: %f Min MVC: %f \n\n", maxMinMVC[0], maxMinMVC[1]);
     fflush(0);
     */
 }

// Pinta un grid de l�neas que abarca un cuadrado de lado width con
// tantas l�nes como indica lines.
void GLWidget::drawSceneGrid(int lines, double width) {

    double incr = width/(double)lines;
    glDisable(GL_LIGHTING);

    glBegin(GL_LINES);
    glColor3f(1.0,1.0,1.0);
    for(int i = 0; i<=lines; i++)
    {
        glVertex3f(-width/2+incr*i+m_ptCenter[0], -width/2+m_ptCenter[1], -5);
        glVertex3f(-width/2+incr*i+m_ptCenter[0], width/2+m_ptCenter[1], -5);
    }

    for(int i = 0; i<=lines; i++)
    {
        glVertex3f(-width/2+m_ptCenter[0], -width/2+incr*i+m_ptCenter[1], -5);
        glVertex3f(width/2+m_ptCenter[0], -width/2+incr*i +m_ptCenter[1], -5);
    }

    glEnd();
}

bool GLWidget::processAllCoords()
{
    //TODO
    /*
    QTime time;
    QTime total;
    time.start();
    total.start();

    //newModeloGC.Clear();
    //vcg::tri::Append<MyMesh, MyMesh>::Mesh(newModeloGC,modeloOriginal, false);
    //gpCalculateGreenCoordinates( modeloOriginal, cage, PerVertGC, PerFaceGC );

    QString texto("Obteniendo Green Coords....");
    parent->ui->infoBar->setText(QString(INFO_STRING).arg(texto));
    m.processGreenCoordinates();
    selectionValuesForColorGC.resize(m.modeloOriginal.vn, 0);

    double timeLapse = time.elapsed()/1000.0;time.start();
    texto = QString("Green: en %1, Obteniendo Harmoni Coords....");
    parent->ui->infoBar->setText(QString(INFO_STRING).arg(texto.arg(timeLapse)));
    m.processHarmonicCoordinates();
    parent->ui->SliceSelector->setMinimum(0);
    parent->ui->SliceSelector->setMaximum(m.HCgrid.dimensions.Y()-1);
    selectionValuesForColorHC.resize(m.modeloOriginal.vn, 0);

    timeLapse = time.elapsed()/1000.0;time.start();
    texto = QString("Harmonic: en %1, Obteniendo Mean Value Coords....");
    parent->ui->infoBar->setText(QString(INFO_STRING).arg(texto.arg(timeLapse)));
    m.processMeanValueCoordinates();
    selectionValuesForColorMVC.resize(m.modeloOriginal.vn, 0);

    // Para luego pintar bien todo.
    timeLapse = time.elapsed()/1000.0;
    double totalLapse = total.elapsed()/1000.0;
    texto = QString("Mean value: en %1, total en %2....");
    parent->ui->infoBar->setText(QString(INFO_STRING).arg(texto.arg(timeLapse)).arg(texto.arg(totalLapse)));

    bGCcomputed = true;
    bHComputed = true;
    bMVCComputed = true;

    deformMesh();
    updateGL();
    */
    return true;

}

bool GLWidget::processGreenCoordinates()
{
    //TODO
    /*
    QTime time;
    time.start();

    //newModeloGC.Clear();
    //vcg::tri::Append<MyMesh, MyMesh>::Mesh(newModeloGC,modeloOriginal, false);
    //gpCalculateGreenCoordinates( modeloOriginal, cage, PerVertGC, PerFaceGC );

    m.processGreenCoordinates();

    // Para luego pintar bien todo.
    selectionValuesForColorGC.resize(m.modeloOriginal.vn, 0);

    double timeLapse = time.elapsed()/1000.0;
    QString texto("Ha tardado %1 segundos en obtener las coordenadas de green.");
    parent->ui->infoBar->setText(QString(INFO_STRING).arg(texto.arg(timeLapse)));
    bGCcomputed = true;

    deformMesh();
    updateGL();
    */
    return true;
}

bool GLWidget::processMeanValueCoordinates()
{

    //TODO
    /*
    QTime time;
    time.start();

    //newModeloGC.Clear();
    //vcg::tri::Append<MyMesh, MyMesh>::Mesh(newModeloGC,modeloOriginal, false);
    //gpCalculateGreenCoordinates( modeloOriginal, cage, PerVertGC, PerFaceGC );

    m.processMeanValueCoordinates();

    // Para luego pintar bien todo.
    selectionValuesForColorMVC.resize(m.modeloOriginal.vn, 0);

    double timeLapse = time.elapsed()/1000.0;
    QString texto("Ha tardado %1 segundos en obtener las coordenadas de mean value.");
    parent->ui->infoBar->setText(QString(INFO_STRING).arg(texto.arg(timeLapse)));
    bMVCComputed = true;

    deformMesh();
    updateGL();
    */
    return true;
}

bool GLWidget::processHarmonicCoordinates()
{
    //TODO
    /*
    QTime time;
    time.start();

    //newModeloHC.Clear();
    //vcg::tri::Append<MyMesh, MyMesh>::Mesh(newModeloHC,modeloOriginal, false);

    //unsigned int resolution = pow(2,7);
    //QString sHCSavedGrid = QDir::currentPath()+"/"+HC_GRID_FILE_NAME;

    //if(QFile(sHCSavedGrid).exists())
    //    loadHarmonicCoordinates(modeloOriginal, HCgrid, PerVertHC, sHCSavedGrid.toStdString());
    //else
//        getHarmonicCoordinates(modeloOriginal, cage, HCgrid, PerVertHC, resolution, sHCSavedGrid.toStdString());

    m.processHarmonicCoordinates();

    parent->ui->SliceSelector->setMinimum(0);
    parent->ui->SliceSelector->setMaximum(m.HCgrid.dimensions.Y()-1);

    selectionValuesForColorHC.resize(m.modeloOriginal.vn, 0);

    double timeLapse = time.elapsed()/1000.0;
    QString texto("Ha tardado %1 segundos en obtener las coordenadas harmonicas.");
    parent->ui->infoBar->setText(QString(INFO_STRING).arg(texto.arg(timeLapse)));
    bHComputed = true;
    deformMesh();
    updateGL();
    */
    return true;
}

bool GLWidget::deformMesh()
{ 

    //TODO
    /*
    if(activeCoords == HARMONIC_COORDS)
    {
        if(!bHComputed)
            return true;

        QTime time;
        time.start();

        if(stillCageAbled && stillCageSelected >= 0)
            m.deformMesh(stillCageSelected, HARMONIC_COORDS );
        else
            m.deformMesh(DYN_CAGE_ID, HARMONIC_COORDS );
        //deformMeshWithHC(modeloOriginal, cage, newModeloHC, newCage, PerVertHC);

        //parent->ui->deformedMeshCheck->setEnabled(true);
        QString texto("y %1 ms en procesar el nuevo modelo con HC.");
        parent->ui->infoBar->setText(QString(INFO_STRING).arg(texto.arg(time.elapsed())));
        return true;
    }
    else if(activeCoords == GREEN_COORDS)
    {
        if(!bGCcomputed)
            return true;

        QTime time;
        time.start();

        if(stillCageAbled && stillCageSelected >= 0)
            m.deformMesh(stillCageSelected, GREEN_COORDS );
        else
            m.deformMesh(DYN_CAGE_ID, GREEN_COORDS );
        //deformMeshWithGC(modeloOriginal, cage, newModeloGC, newCage, PerVertGC, PerFaceGC);
        //parent->ui->deformedMeshCheck->setEnabled(true);
        QString texto("y %1 ms en procesar el nuevo modelo con GC.");
        parent->ui->infoBar->setText(QString(INFO_STRING).arg(texto.arg(time.elapsed())));
        return true;
    }
    else if(activeCoords == MEANVALUE_COORDS)
    {
        if(!bMVCComputed)
            return true;

        QTime time;
        time.start();

        if(stillCageAbled && stillCageSelected >= 0)
            m.deformMesh(stillCageSelected, MEANVALUE_COORDS );
        else
            m.deformMesh(DYN_CAGE_ID, MEANVALUE_COORDS );
        //deformMeshWithGC(modeloOriginal, cage, newModeloGC, newCage, PerVertGC, PerFaceGC);
        //parent->ui->deformedMeshCheck->setEnabled(true);
        QString texto("y %1 ms en procesar el nuevo modelo con MVC.");
        parent->ui->infoBar->setText(QString(INFO_STRING).arg(texto.arg(time.elapsed())));
        return true;
    }

    */
    return false;
}

void GLWidget::showDeformedModelSlot()
{
    showDeformedModel = parent->ui->deformedMeshCheck->isChecked();
    updateGL();
}

void GLWidget::updateGridRender()
{
    for(unsigned int i = 0; i< escena->visualizers.size(); i++)
    {
        if(escena->visualizers[i]->iam == GRIDRENDERER_NODE)
        {

            gridRenderer* gr = ((gridRenderer*)escena->visualizers[i]);
            gr->m_bShowAllGrid = parent->ui->allGrid_button->isChecked();
            gr->m_bBorders = parent->ui->GridDraw_interior->isChecked();
            gr->m_bShow_exterior = parent->ui->GridDraw_exterior->isChecked();
            gr->m_bShow_boundary = parent->ui->GridDraw_boundary->isChecked();

            gr->grid->res = parent->ui->gridResolutionIn->text().toInt();
			gr->grid->worldScale = parent->ui->sceneScale->text().toInt();

            gr->desiredVertex = parent->ui->DistancesVertSource->text().toInt();

            gr->setSliceXY(parent->ui->SliceSelectorXY->value());
            gr->setSliceXZ(parent->ui->SliceSelectorXZ->value());

            gr->propagateDirtyness();
       } 
    }
}

void GLWidget::showHCoordinatesSlot()
{
    //m_bShowHCGrid = parent->ui->HCGridDraw->isChecked();
    //m_bShowHCGrid_interior = parent->ui->GridDraw_interior->isChecked();
    //m_bShowHCGrid_exterior = parent->ui->GridDraw_exterior->isChecked();
    //m_bShowHCGrid_boundary = parent->ui->GridDraw_boundary->isChecked();
    //m_bShowAllGrid = parent->ui->allGrid_button->isChecked();

    //m_bDrawHCInfluences = parent->ui->drawInfluences_check->isChecked();

    //if(!m_bShowAllGrid)
    //    parent->ui->SliceSelector->setEnabled(true);

    //ChangeSlice(parent->ui->SliceSelector->value());
    //updateGL();
}

void GLWidget::ChangeStillCage(int id)
{
    stillCageSelected = parent->ui->cagesComboBox->itemData(id).toInt();
}

void GLWidget::ChangeViewingMode(int)
{

}

 void GLWidget::UpdateVertexSource(int id)
 {
	 escena->desiredVertex = id;
	 paintModelWithData();
	 //paintPlaneWithData();

     //updateGridRender();

     //TODO
     /*
    double vmin = 9999999, vmax = -9999999;
    for(int i = 0; i < m.modeloOriginal.vn; i++)
    {
        vmin = min(BHD_distancias[i][id], vmin);
        vmax = max(BHD_distancias[i][id], vmax);
    }

    printf("Distancias-> max:%f min:%f\n", vmax, vmin); fflush(0);

    vertexColors.resize(m.modeloOriginal.vn);
    for(int i = 0; i < m.modeloOriginal.vn; i++)
    {
        vertexColors[i] = GetColour(BHD_distancias[i][id], vmin, vmax);
    }

    colorsLoaded = true;

    updateGL();
    */
 }

 void GLWidget::setContextMode(contextMode ctx)
 {
    selMgr.ctx = ctx;
 }

 void GLWidget::ChangeSliceXZ(int slice)
 {
     updateGridRender();
 }

void GLWidget::ChangeSliceXY(int slice)
{
    updateGridRender();

    //TODO
    /*
    currentDrawingSlice = slice;

    if(!bHComputed)
        return;

    sliceValues.resize(m.HCgrid.dimensions.X());

    for(unsigned int i = 0; i< sliceValues.size(); i++)
    {
        sliceValues[i].resize(m.HCgrid.dimensions.Z());
        for(unsigned int j = 0; j< sliceValues[i].size(); j++)
        {
            sliceValues[i][j] = 0;
        }
    }

    for(unsigned int i = 0; i< sliceValues.size(); i++)
    {
        for(unsigned int j = 0; j< sliceValues[i].size(); j++)
        {
            float valueSum = 0;
            for (QList<int>::const_iterator it=selection_.begin(), end=selection_.end(); it != end; ++it)
            {
                valueSum += m.HCgrid.cells[i][currentDrawingSlice][j]->weights[*it];
            }

            if (valueSum != 0)
            {
                sliceValues[i][j] = valueSum;
                //printf("Valor de %d %d : %f", i, j, sliceValues[i][j]); fflush(0);
            }
        }
    }
    */

}

void GLWidget::active_GC_vs_HC(int tab)
{
    //TODO
    /*
    if(tab == 0) // GC
    {
        activeCoords = GREEN_COORDS;
    }
    else if(tab == 1)
    {
        activeCoords = HARMONIC_COORDS;
    }
    else if(tab == 2)
    {
        activeCoords = MEANVALUE_COORDS;
    }
    */
}

void GLWidget::startManipulation()
{
  Vec averagePosition;
  ManipulatedFrameSetConstraint* mfsc = (ManipulatedFrameSetConstraint*)(manipulatedFrame()->constraint());
  mfsc->clearSet();

  for (QList<int>::const_iterator it=selection_.begin(), end=selection_.end(); it != end; ++it)
    {
      mfsc->addObjectToSet(objects_[*it]);
      averagePosition += objects_[*it]->frame.position();
    }

  if (selection_.size() > 0)
    manipulatedFrame()->setPosition(averagePosition / selection_.size());
}


//   S e l e c t i o n   t o o l s

void GLWidget::addIdToSelection(int id)
{
  if (!selection_.contains(id))
    selection_.push_back(id);
}

void GLWidget::removeIdFromSelection(int id)
{
  selection_.removeAll(id);
}

void GLWidget::drawSelectionRectangle() const
{
  startScreenCoordinatesSystem();
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND);

  glColor4f(0.0, 0.0, 0.3f, 0.3f);
  glBegin(GL_QUADS);
  glVertex2i(rectangle_.left(),  rectangle_.top());
  glVertex2i(rectangle_.right(), rectangle_.top());
  glVertex2i(rectangle_.right(), rectangle_.bottom());
  glVertex2i(rectangle_.left(),  rectangle_.bottom());
  glEnd();

  glLineWidth(2.0);
  glColor4f(0.4f, 0.4f, 0.5f, 0.5f);
  glBegin(GL_LINE_LOOP);
  glVertex2i(rectangle_.left(),  rectangle_.top());
  glVertex2i(rectangle_.right(), rectangle_.top());
  glVertex2i(rectangle_.right(), rectangle_.bottom());
  glVertex2i(rectangle_.left(),  rectangle_.bottom());
  glEnd();

  glDisable(GL_BLEND);
  glEnable(GL_LIGHTING);
  stopScreenCoordinatesSystem();
}

void GLWidget::endSelection(const QPoint&)
{
  // Flush GL buffers
  glFlush();

  // Get the number of objects that were seen through the pick matrix frustum. Reset GL_RENDER mode.
  GLint nbHits = glRenderMode(GL_RENDER);

  if (nbHits > 0)
    {
      // Interpret results : each object created 4 values in the selectBuffer().
      // (selectBuffer())[4*i+3] is the id pushed on the stack.
      for (int i=0; i<nbHits; ++i)
        switch (selectionMode_)
          {
          case ADD    : addIdToSelection((selectBuffer())[4*i+3]); break;
          case REMOVE : removeIdFromSelection((selectBuffer())[4*i+3]);  break;
          default : break;
          }
    }
  selectionMode_ = NONE;
}

 //   C u s t o m i z e d   m o u s e   e v e n t s

 void GLWidget::mousePressEvent(QMouseEvent* e)
 {
   // Start selection. Mode is ADD with Shift key and TOGGLE with Alt key.
   rectangle_ = QRect(e->pos(), e->pos());

   if ((e->button() == Qt::LeftButton) && (e->modifiers() == Qt::ShiftModifier))
   {
     selectionMode_ = ADD;
   }
   else
     if ((e->button() == Qt::LeftButton) && (e->modifiers() == Qt::AltModifier))
     {
       selectionMode_ = REMOVE;
     }
      else
       {
        // if (e->modifiers() == Qt::ControlModifier)
        //    startManipulation();

            QGLViewer::mousePressEvent(e);
       }

 }

 void GLWidget::mouseMoveEvent(QMouseEvent* e)
 {
   if (selectionMode_ != NONE)
     {
       // Updates rectangle_ coordinates and redraws rectangle
       rectangle_.setBottomRight(e->pos());
       updateGL();
     }
   else
     QGLViewer::mouseMoveEvent(e);
 }

 void GLWidget::mouseReleaseEvent(QMouseEvent* e)
 {
   if (selectionMode_ != NONE)
     {
       // Actual selection on the rectangular area.
       // Possibly swap left/right and top/bottom to make rectangle_ valid.
       rectangle_ = rectangle_.normalized();
       // Define selection window dimensions
       setSelectRegionWidth(rectangle_.width());
       setSelectRegionHeight(rectangle_.height());
       // Compute rectangle center and perform selection
       select(rectangle_.center());
       // Update display to show new selected objects

       printf("Seleccion:\n");
       for (QList<int>::const_iterator it=selection_.begin(), end=selection_.end(); it != end; ++it)
       {
           printf("%d ", *it);
       }
       printf("\n\n"); fflush(0);

       //startManipulation();
       updateGL();
     }
   else
     QGLViewer::mouseReleaseEvent(e);
 }


