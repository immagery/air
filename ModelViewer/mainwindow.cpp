//#include <cmath>

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QToolButton>
#include <QToolBar>
#include <QDir>
#include <QFileDialog>

#include <QTreeView>
#include <QStandardItem>

#include "treeitem.h"
#include "treemodel.h"

#include "outliner.h"
#include "modelo.h"

#include "util.h"

#include "globalDefinitions.h"

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->glCustomWidget->parent = this;

    // conexiones
    connect(ui->action_importModel, SIGNAL(triggered()), this, SLOT(ImportNewModel()) );
    connect(ui->actionAction_openScene, SIGNAL(triggered()), this, SLOT(OpenNewScene()));

    connect(ui->import_cage_s, SIGNAL(triggered()), this, SLOT(ImportCages()) );
    connect(ui->import_distances, SIGNAL(triggered()), this, SLOT(ImportDistances()) );

    connect(ui->shadingModeSelection, SIGNAL(currentIndexChanged(int)), this, SLOT(ShadingModeChange(int)) );

    connect(ui->colorLayersCheck, SIGNAL(toggled(bool)), this, SLOT(EnableColorLayer(bool)) );
    connect(ui->ColorLayerSeletion, SIGNAL(valueChanged(int)), this, SLOT(ChangeColorLayerValue(int)) );

    connect(ui->actionImportSegementation, SIGNAL(triggered()), this, SLOT(DoAction()) );

    //connect(ui->prop_function_updt, SIGNAL(released()), this, SLOT(ChangeSourceVertex()));

    connect(ui->DistancesVertSource, SIGNAL(valueChanged(int)), this, SLOT(distancesSourceValueChange(int)));

    connect(ui->actionExit, SIGNAL(triggered()), this, SLOT(CloseApplication()) );
    connect(ui->processGC,  SIGNAL(triggered()), ui->glCustomWidget, SLOT(processGreenCoordinates()) );
    connect(ui->processHC,  SIGNAL(triggered()), ui->glCustomWidget, SLOT(processHarmonicCoordinates()));
    connect(ui->processMVC, SIGNAL(triggered()), ui->glCustomWidget, SLOT(processMeanValueCoordinates()));
    connect(ui->processAll, SIGNAL(triggered()), ui->glCustomWidget, SLOT(processAllCoords()));
    connect(ui->deformedMeshCheck, SIGNAL(released()), ui->glCustomWidget, SLOT(showDeformedModelSlot()));

    connect(ui->cagesComboBox, SIGNAL(currentIndexChanged(int)), ui->glCustomWidget, SLOT(ChangeStillCage(int)));
    connect(ui->enableStillCage, SIGNAL(toggled(bool)), this, SLOT(enableStillCage(bool)));

    connect(ui->HCGridDraw, SIGNAL(released()), ui->glCustomWidget, SLOT(showHCoordinatesSlot()));
    connect(ui->GridDraw_boundary, SIGNAL(released()), ui->glCustomWidget, SLOT(updateGridRender()));

    // Actualizaciones del grid.
    connect(ui->GridDraw_interior, SIGNAL(released()), ui->glCustomWidget, SLOT(updateGridRender()));
    connect(ui->GridDraw_exterior, SIGNAL(released()), ui->glCustomWidget, SLOT(updateGridRender()));
    connect(ui->allGrid_button, SIGNAL(released()), ui->glCustomWidget, SLOT(updateGridRender()));
    connect(ui->gridSlices_button, SIGNAL(released()), ui->glCustomWidget, SLOT(updateGridRender()));
    connect(ui->SliceSelectorXY, SIGNAL(valueChanged(int)), ui->glCustomWidget, SLOT(ChangeSliceXY(int)));
    connect(ui->SliceSelectorXZ, SIGNAL(valueChanged(int)), ui->glCustomWidget, SLOT(ChangeSliceXZ(int)));

    connect(ui->voxelization_btn, SIGNAL(released()), ui->glCustomWidget, SLOT(computeProcess()));
	connect(ui->nextStep_button, SIGNAL(released()), ui->glCustomWidget, SLOT(nextProcessStep()));
	connect(ui->allNextStep_button, SIGNAL(released()), ui->glCustomWidget, SLOT(allNextProcessSteps()));

    connect(ui->prop_function_updt, SIGNAL(released()), ui->glCustomWidget, SLOT(PropFunctionConf()));

    connect(ui->paintModel_btn, SIGNAL(released()), ui->glCustomWidget, SLOT(paintModelWithGrid()));
    connect(ui->metricUsedCheck, SIGNAL(released()), ui->glCustomWidget, SLOT(PropFunctionConf()));

    connect(ui->drawInfluences_check, SIGNAL(released()), ui->glCustomWidget, SLOT(showHCoordinatesSlot()));

    connect(ui->coordTab, SIGNAL(currentChanged(int)), ui->glCustomWidget, SLOT(active_GC_vs_HC(int)));

    connect(ui->glCustomWidget, SIGNAL(updateSceneView()), this, SLOT(updateSceneView()));
    connect(ui->outlinerView, SIGNAL(clicked(QModelIndex)), this, SLOT(selectObject(QModelIndex)));

    connect(ui->actionMove, SIGNAL(triggered()), this, SLOT(toogleMoveTool()));
    connect(ui->actionSelection, SIGNAL(triggered()), this, SLOT(toogleSelectionTool()));
    connect(ui->actionRotate, SIGNAL(triggered()), this, SLOT(toogleRotateTool()));
    connect(ui->visibility_btn, SIGNAL(toggled(bool)), this, SLOT(toogleVisibility(bool)));

	connect(ui->actionDoTests, SIGNAL(triggered()), this, SLOT(LaunchTests()));

    connect(ui->segmentation_btn, SIGNAL(toggled(bool)), this, SLOT(toogleToShowSegmentation(bool)));
    connect(ui->DataVisualizationCombo, SIGNAL(currentIndexChanged(int)),this, SLOT(DataVisualizationChange(int)) );

    connect(ui->exportWeights_btn, SIGNAL(released()), ui->glCustomWidget, SLOT(exportWeightsToMaya()));

    connect(ui->expansionSlider, SIGNAL(sliderReleased()), this, SLOT(changeExpansionSlider()));
    connect(ui->expansionSlider, SIGNAL(valueChanged(int)), this, SLOT(updateExpansionSlidervalue(int)));

	connect(ui->thresholdSlider, SIGNAL(valueChanged(int)), this, SLOT(updateThresholdSlidervalue(int)));
	connect(ui->threshold_enable, SIGNAL(toggled(bool)), this, SLOT(enableThreshold(bool)));
	connect(ui->threshold_enable_adaptative, SIGNAL(toggled(bool)), this, SLOT(enableAdaptativeThreshold(bool)));
	

    connect(ui->smoothPropagationSlider, SIGNAL(sliderReleased()), this, SLOT(changeSmoothSlider()));
    connect(ui->smoothPropagationSlider, SIGNAL(valueChanged(int)), this, SLOT(updateSmoothSlidervalue(int)));

    connect(ui->glCustomWidget, SIGNAL(jointDataShow(float, int)), this , SLOT(jointDataUpdate(float,int)));

	connect(ui->ip_axisX, SIGNAL(valueChanged(int)), this, SLOT(changeInteriorPointPosition()));
	connect(ui->ip_axisY, SIGNAL(valueChanged(int)), this, SLOT(changeInteriorPointPosition()));
	connect(ui->ip_axisZ, SIGNAL(valueChanged(int)), this, SLOT(changeInteriorPointPosition()));

	connect(ui->paintModel, SIGNAL(clicked()), this, SLOT(updateModelColors()));
	connect(ui->paintPlane, SIGNAL(clicked()), this, SLOT(updateClipingPlaneColor()));

	connect(ui->drawPlaneCheck, SIGNAL(clicked()), this, SLOT(updateClipingPlaneData()));
	connect(ui->PlaneOrientX, SIGNAL(clicked()), this, SLOT(updateClipingPlaneData()));
	connect(ui->planeOrientY, SIGNAL(clicked()), this, SLOT(updateClipingPlaneData()));
	connect(ui->planeOrientZ, SIGNAL(clicked()), this, SLOT(updateClipingPlaneData()));
	connect(ui->PlaneDataCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(changeVisModeForPlane(int)));
	
	connect(ui->dataSource, SIGNAL(valueChanged(int)), this, SLOT(distancesSourceValueChange(int)));
	connect(ui->positionPlaneSlider, SIGNAL(sliderMoved(int)), this, SLOT(changeSelPointForPlane(int)));
}


void MainWindow::changeSlider(int)
{

}

void MainWindow::ImportNewModel()
{
    QFileDialog inFileDialog(0, "Selecciona un fichero", ui->glCustomWidget->sPathGlobal, "*.off *.txt");
    inFileDialog.setFileMode(QFileDialog::ExistingFile);
    QStringList fileNames;
     if (inFileDialog.exec())
         fileNames = inFileDialog.selectedFiles();

    if(fileNames.size() == 0)
        return;

    QFileInfo sPathAux(fileNames[0]);
    QString aux = sPathAux.canonicalPath();
    QString sModelPath = aux;
    int ini = fileNames[0].indexOf("/",aux.length());
    aux = fileNames[0].right(fileNames[0].length()-ini);
    QString sModelPrefix = aux.left(aux.length()-4);

    ui->glCustomWidget->readModel(fileNames[0].toStdString(), sModelPrefix.toStdString(), sModelPath.toStdString());
}

void MainWindow::OpenNewScene()
{
    QFileDialog inFileDialog(0, "Selecciona un fichero", ui->glCustomWidget->sPathGlobal, "*.txt");
    inFileDialog.setFileMode(QFileDialog::ExistingFile);
    QStringList fileNames;
     if (inFileDialog.exec())
         fileNames = inFileDialog.selectedFiles();

    if(fileNames.size() == 0)
        return;

    QFileInfo sPathAux(fileNames[0]);
    QString aux = sPathAux.canonicalPath();
    QString sModelPath = aux;
    int ini = fileNames[0].indexOf("/",aux.length());
    aux = fileNames[0].right(fileNames[0].length()-ini);
    QString sModelPrefix = aux.left(aux.length()-4);

    ui->glCustomWidget->readScene(fileNames[0].toStdString(), sModelPrefix.toStdString(), sModelPath.toStdString());
}

void MainWindow::LaunchTests()
{
    QFileDialog inFileDialog(0, "Selecciona un fichero", ui->glCustomWidget->sPathGlobal, "*.txt");
    inFileDialog.setFileMode(QFileDialog::ExistingFile);
    QStringList fileNames;
     if (inFileDialog.exec())
         fileNames = inFileDialog.selectedFiles();

    if(fileNames.size() == 0)
        return;

    QFileInfo sPathAux(fileNames[0]);
    QString aux = sPathAux.canonicalPath();
    QString sModelPath = aux;
    int ini = fileNames[0].indexOf("/",aux.length());
    aux = fileNames[0].right(fileNames[0].length()-ini);
    QString sModelPrefix = aux.left(aux.length()-4);

	ui->glCustomWidget->doTests(fileNames[0].toStdString(), sModelPrefix.toStdString(), sModelPath.toStdString());
}

void MainWindow::updateSmoothSlidervalue(int)
{
    float valueAux = ui->smoothPropagationSlider->value();
    float value = 0;
    if(valueAux <= 100)
        value = ((float)ui->smoothPropagationSlider->value())/100.0;
    else
    {
        value = (((valueAux-100)/100)*9)+1.0;
    }

    ui->smoothPropagationEdit->setText(QString("%1").arg(value));
}

void MainWindow::changeSmoothSlider()
{
    float valueAux = ui->smoothPropagationSlider->value();
    float value = 0;
    if(valueAux <= 100)
        value = ((float)ui->smoothPropagationSlider->value())/100.0;
    else
    {
        value = (((valueAux-100)/100)*2)+1.0;
    }

    ui->smoothPropagationEdit->setText(QString("%1").arg(value));
    ui->glCustomWidget->changeSmoothPropagationDistanceRatio(value);
}

void MainWindow::updateThresholdSlidervalue(int value)
{
	ui->threshold_edit->setText(QString("%1").arg((float)value/100.0));

	if(ui->threshold_enable->isChecked())
		ui->glCustomWidget->setThreshold((float)value/100.0);
	else
		ui->glCustomWidget->setThreshold( -10);
}
void MainWindow::enableThreshold(bool toogle)
{
	int value = ui->thresholdSlider->value();

	if(ui->threshold_enable->isChecked())
		ui->glCustomWidget->setThreshold((float)value/100.0);
	else
		ui->glCustomWidget->setThreshold( -10);
}

void MainWindow::enableAdaptativeThreshold(bool toogle)
{
	int value = ui->thresholdSlider->value();

	if(!ui->threshold_enable_adaptative->isChecked())
		enableThreshold(ui->threshold_enable->isChecked());
	else
		ui->glCustomWidget->setThreshold(0);
}

void MainWindow::updateExpansionSlidervalue(int)
{
    float valueAux = ui->expansionSlider->value();
    float value = 0;
    if(valueAux <= 100)
        value = ((float)ui->expansionSlider->value())/100.0;
    else
    {
        value = (((valueAux-100)/100)*9)+1.0;
    }

    ui->expansionValueEdit->setText(QString("%1").arg(value));
}

void MainWindow::changeExpansionSlider()
{
    float valueAux = ui->expansionSlider->value();
    float value = 0;
    if(valueAux <= 100)
        value = ((float)ui->expansionSlider->value())/100.0;
    else
    {
        value = (((valueAux-100)/100)*2)+1.0;
    }

    ui->expansionValueEdit->setText(QString("%1").arg(value));
    ui->glCustomWidget->changeExpansionFromSelectedJoint(value);
}

void MainWindow::changeVisModeForPlane(int)
{	
	updateClipingPlaneData();
}

void MainWindow::changeSelPointForPlane(int)
{
	updateClipingPlaneData();
}

void MainWindow::updateClipingPlaneData()
{
	double sliderPos = (float)ui->positionPlaneSlider->value()/1000.0;
	int visCombo = ui->PlaneDataCombo->currentIndex();
	int selectedVertex = ui->dataSource->text().toInt();
	bool checked = ui->drawPlaneCheck->checkState() == Qt::Checked;
	ui->positionPlaneEdit->setText(QString("%1").arg(sliderPos, 3, 'g', 3));

	int orient = 0;
	if(ui->planeOrientY->isChecked())
		orient = 1;
	else if(ui->planeOrientZ->isChecked())
		orient = 2;

	ui->glCustomWidget->setPlaneData(checked, selectedVertex,visCombo, sliderPos, orient);
}

void MainWindow::updateClipingPlaneColor()
{
	ui->glCustomWidget->paintPlaneWithData(true);
}

void MainWindow::updateModelColors()
{
	ui->glCustomWidget->paintModelWithData();
}

void MainWindow::changeInteriorPointPosition()
{
    float valueAuxX = ui->ip_axisX->value();
	float valueAuxY = ui->ip_axisY->value();
	float valueAuxZ = ui->ip_axisZ->value();

	ui->axisX_edit->setText(QString("%1").arg(valueAuxX));
	ui->axisY_edit->setText(QString("%1").arg(valueAuxY));
	ui->axisZ_edit->setText(QString("%1").arg(valueAuxZ));

	ui->glCustomWidget->interiorPoint = Point3d(valueAuxX,valueAuxY,valueAuxZ);
	ui->glCustomWidget->setPlanePosition(valueAuxX,valueAuxY,valueAuxZ);
}

void MainWindow::jointDataUpdate(float fvalue, int id)
{
    if(fvalue <=1)
    {
		ui->expansionSlider->setValue((int)round(fvalue*100));
    }
    else
    {
        int value = ((int)round((fvalue-1)/9*100)+100);
        ui->expansionSlider->setValue(value);
    }

    ui->expansionValueEdit->setText(QString("%1").arg(fvalue));

    ui->DistancesVertSource->setValue(id);
    distancesSourceValueChange(id);
}

void MainWindow::ImportCages()
{
    /*
    QFileDialog inFileDialog(0, "Selecciona un fichero", ui->glCustomWidget->sPathGlobal, "*.off , *.txt");
    inFileDialog.setFileMode(QFileDialog::ExistingFile);
    QStringList fileNames;
     if (inFileDialog.exec())
         fileNames = inFileDialog.selectedFiles();

    if(fileNames.size() == 0)
        return;

    ui->glCustomWidget->readCage(fileNames[0], ui->glCustomWidget->m);
    */

    printf("Function deprecated\n"); fflush(0);
}

void MainWindow::toogleMoveTool()
{
    if(toolSelected == T_MOVETOOL)
    {
        ui->actionMove->setChecked(true);
        return;
    }

    ui->actionRotate->setChecked(false);
    ui->actionSelection->setChecked(false);
    changeTool(T_MOVETOOL);
}

void MainWindow::toogleRotateTool()
{
    if(toolSelected == T_ROTATETOOL)
    {
       ui->actionRotate->setChecked(true);
        return;
    }

    ui->actionMove->setChecked(false);
    ui->actionSelection->setChecked(false);
    changeTool(T_ROTATETOOL);

}

void MainWindow::toogleSelectionTool()
{
    if(toolSelected == T_SELECTTOOL)
    {
        ui->actionSelection->setChecked(true);
        return;
    }

    ui->actionMove->setChecked(false);
    ui->actionRotate->setChecked(false);
    changeTool(T_SELECTTOOL);
}

void MainWindow::changeTool(toolmode newtool)
{
    switch(newtool)
    {
    case T_MOVETOOL:
        ui->glCustomWidget->setContextMode(CTX_MOVE);
        break;
    case T_SELECTTOOL:
        ui->glCustomWidget->setContextMode(CTX_SELECTION);
        break;
    case T_ROTATETOOL:
        ui->glCustomWidget->setContextMode(CTX_ROTATION);
        break;
    default:
        printf("Hay algún problema con la seleccion de contexto.\n"); fflush(0);
        ui->glCustomWidget->setContextMode(CTX_SELECTION);
        break;
    }

    printf("ha habido un cambio\n"); fflush(0);

}

void MainWindow::ShadingModeChange(int option)
{
    ui->glCustomWidget->cleanShadingVariables();
    if (option == 0)
    {
        ui->glCustomWidget->ShadingModeFlag = SH_MODE_SMOOTH;
        ui->glCustomWidget->cleanShadingVariables();
    }

    if (option == 1)
    {
        ui->glCustomWidget->ShadingModeFlag = SH_MODE_FLAT;
        ui->glCustomWidget->cleanShadingVariables();
    }

    if (option == 2)
    {
        ui->glCustomWidget->ShadingModeFlag = SH_MODE_SMOOTH;
        ui->glCustomWidget->toogleModelToXRay();
    }

    if (option == 3)
    {
        ui->glCustomWidget->ShadingModeFlag = SH_MODE_FLAT;
        ui->glCustomWidget->toogleModelToLines();
    }
}

void MainWindow::toogleVisibility(bool toogle)
{
   ui->glCustomWidget->toogleVisibility(toogle);
}

void MainWindow::DataVisualizationChange(int mode)
{
    ui->glCustomWidget->changeVisualizationMode(mode);
}

void MainWindow::toogleToShowSegmentation(bool toogle)
{
   ui->glCustomWidget->toogleToShowSegmentation(toogle);
}

void MainWindow::EnableColorLayer(bool _b)
{
    ui->glCustomWidget->colorLayers = _b;
        if (_b) ui->glCustomWidget->colorLayerIdx = ui->ColorLayerSeletion->value();
}

void MainWindow::ChangeColorLayerValue(int value)
{
    ui->glCustomWidget->colorLayerIdx = value;
}

void MainWindow::updateSceneView(TreeItem* treeitem, treeNode* treenode)
{
    for(unsigned int i = 0; i< treenode->childs.size(); i++)
    {
        QList<QVariant> columnData;
        columnData << QString(((treeNode*)treenode->childs[i])->sName.c_str());// << ((treeNode*)treenode->childs[i])->nodeId;

        TreeItem* elem = new TreeItem(columnData, treeitem);
        elem->sName = ((treeNode*)treenode->childs[i])->sName;
        elem->item_id = ((treeNode*)treenode->childs[i])->nodeId;
        elem->type = ((treeNode*)treenode->childs[i])->type;

        updateSceneView(elem,treenode->childs[i]);

        treeitem->appendChild(elem);
    }

}

void MainWindow::updateSceneView()
{
    //treeViewModel->clear(); // Limpiamos la lista.

    TreeModel* treeViewModel;
    treeViewModel = new TreeModel();
    ui->outlinerView->setModel(treeViewModel);

    treeNode* root = new treeNode();
    outliner outl(ui->glCustomWidget->escena);
    outl.getSceneTree(root);

    QList<TreeItem*> parents;
    parents << treeViewModel->rootItem;
    updateSceneView(parents.back(), root);

    emit ui->outlinerView->dataChanged(ui->outlinerView->rootIndex(), ui->outlinerView->rootIndex());
    repaint();
    // Hay que actualizar la vista para que aparezca todo.
}


void MainWindow::selectObject(QModelIndex idx)
{
    //TreeModel* model = (TreeModel*)ui->outlinerView->model();

    if (!idx.isValid()) return;

    TreeItem *item = static_cast<TreeItem*>(idx.internalPointer());

    //item->checked = !item->checked;
    //return item->checked;


    if(item->item_id == 0)
        return;

    // Listado de elementos seleccionados.
    vector<unsigned int > lst;
    lst.push_back(item->item_id);
    ui->glCustomWidget->selectElements(lst);
	//ui->glCustomWidget->updateGridVisualization();
	ui->glCustomWidget->paintModelWithData();

    printf("Selected: %s con id: %d\n", item->sName.c_str(), item->item_id); fflush(0);

    repaint();
}

void MainWindow::ImportDistances()
{
    QFileDialog inFileDialog(0, "Selecciona un fichero", ui->glCustomWidget->sPathGlobal, "*.txt");
    inFileDialog.setFileMode(QFileDialog::ExistingFile);
    QStringList fileNames;
     if (inFileDialog.exec())
         fileNames = inFileDialog.selectedFiles();

    if(fileNames.size() == 0)
        return;

    ui->glCustomWidget->readDistances(fileNames[0]);
}

void MainWindow::DoAction()
{
    QFileDialog inFileDialog(0, "Selecciona un fichero", ui->glCustomWidget->sPathGlobal, "*.txt");
    inFileDialog.setFileMode(QFileDialog::ExistingFile);
    QStringList fileNames;
    if (inFileDialog.exec())
        fileNames = inFileDialog.selectedFiles();

    if(fileNames.size() == 0)
        return;

    ui->glCustomWidget->importSegmentation(fileNames[0]);
}

void MainWindow::ChangeSourceVertex()
{
    int ind = ui->DistancesVertSource->text().toInt();
    ui->glCustomWidget->UpdateVertexSource(ind);
}

void MainWindow::distancesSourceValueChange(int ind)
{
    ui->glCustomWidget->UpdateVertexSource(ind);
}

void MainWindow::enableStillCage(bool state)
{
    ui->cagesComboBox->setEnabled(ui->enableStillCage->isEnabled());
    ui->glCustomWidget->stillCageAbled = ui->enableStillCage->isEnabled();

    if(ui->cagesComboBox->count()>0)
        ui->glCustomWidget->ChangeStillCage(ui->cagesComboBox->currentIndex());
}

void MainWindow::CloseApplication()
{
    exit(0);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::keyPressEvent(QKeyEvent* event)
{
    int current = 0;
    bool checked = false;
    switch(event->key())
    {
    case Qt::Key_Up:
        current = ui->SliceSelectorXZ->value();
        if(current + 5 <= ui->SliceSelectorXZ->maximum())
        {
            ui->SliceSelectorXZ->setValue(current+5);
            ui->glCustomWidget->ChangeSliceXZ(current+5);
        }
        break;
    case Qt::Key_Down:
        current = ui->SliceSelectorXZ->value();
        if(current - 5 >= ui->SliceSelectorXZ->minimum())
        {
            ui->SliceSelectorXZ->setValue(current-5);
            ui->glCustomWidget->ChangeSliceXZ(current-5);
        }
        break;
    case Qt::Key_Right:
        current = ui->SliceSelectorXY->value();
        if(current + 5 <= ui->SliceSelectorXY->maximum())
        {
            ui->SliceSelectorXY->setValue(current+5);
            ui->glCustomWidget->ChangeSliceXY(current+5);
        }
        break;
    case Qt::Key_Left:
        current = ui->SliceSelectorXY->value();
        if(current - 5 >= ui->SliceSelectorXY->minimum())
        {
            ui->SliceSelectorXY->setValue(current-5);
            ui->glCustomWidget->ChangeSliceXY(current-5);
        }
        break;

    case Qt::Key_Plus:
        current = ui->DistancesVertSource->value();
        ui->DistancesVertSource->setValue(current+1);
        distancesSourceValueChange(current+1);
        break;

    case Qt::Key_Minus:
        current = ui->DistancesVertSource->value();
        if(current >=1)
        {
            ui->DistancesVertSource->setValue(current-1);
            distancesSourceValueChange(current-1);
        }
        break;

    case Qt::Key_0:
        checked = ui->visibility_btn->isChecked();
        toogleVisibility(!checked);
        ui->visibility_btn->setChecked(!checked);
        break;
    case Qt::Key_1:
        DataVisualizationChange(VIS_LABELS);
        ui->DataVisualizationCombo->setCurrentIndex(VIS_LABELS);
        break;
    case Qt::Key_2:
        DataVisualizationChange(VIS_SEGMENTATION);
        ui->DataVisualizationCombo->setCurrentIndex(VIS_SEGMENTATION);
        break;
    case Qt::Key_3:
        DataVisualizationChange(VIS_BONES_SEGMENTATION);
        ui->DataVisualizationCombo->setCurrentIndex(VIS_BONES_SEGMENTATION);
        break;
	case Qt::Key_4:
		DataVisualizationChange(VIS_SEG_PASS);
        ui->DataVisualizationCombo->setCurrentIndex(VIS_SEG_PASS);
        break;
	case Qt::Key_5:
        DataVisualizationChange(VIS_WEIGHTS);
        ui->DataVisualizationCombo->setCurrentIndex(VIS_WEIGHTS);
        break;
    case Qt::Key_6:
        ShadingModeChange(0);
        ui->shadingModeSelection->setCurrentIndex(0);
        ui->infoData->setText("Smooth shading mode");
        break;
    case Qt::Key_7:
        ShadingModeChange(1);
        ui->shadingModeSelection->setCurrentIndex(1);
        ui->infoData->setText("Flat shading Mode");
        break;
    case Qt::Key_8:
        ShadingModeChange(2);
        ui->shadingModeSelection->setCurrentIndex(2);
        ui->infoData->setText("Blend shading mode");
        break;
    case Qt::Key_9:
        ShadingModeChange(3);
        ui->shadingModeSelection->setCurrentIndex(3);
        ui->infoData->setText("Lines shading mode");
        break;
    default:
        break;
    }
}
