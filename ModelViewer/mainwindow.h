#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QModelIndex>

class TreeModel;
class TreeItem;
class treeNode;

namespace Ui {
    class MainWindow;
}

enum toolmode{T_MOVETOOL, T_ROTATETOOL, T_SELECTTOOL};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
    void changeSlider(int);
    void OpenNewScene();
	void LaunchTests();
    void ImportNewModel();
    void CloseApplication();
    void enableStillCage(bool);

    void ImportCages();
    void ImportDistances();

    void ShadingModeChange(int option);

    void DataVisualizationChange(int idx);
    void toogleVisibility(bool toogle);
    void toogleToShowSegmentation(bool toogle);

    void EnableColorLayer(bool _b);
    void ChangeColorLayerValue(int value);

    void DoAction();
    void ChangeSourceVertex();
    void distancesSourceValueChange(int);

    void updateSceneView();
    void updateSceneView(TreeItem* treeitem, treeNode* treenode);
    void selectObject(QModelIndex idx);

    void toogleMoveTool();
    void toogleRotateTool();
    void toogleSelectionTool();
    void changeTool(toolmode newtool);

    void changeSmoothSlider();
    void updateSmoothSlidervalue(int);

    void changeExpansionSlider();
	void changeInteriorPointPosition();
	void updateClipingPlaneColor();
	void updateModelColors();
	void updateClipingPlaneData();
    void updateExpansionSlidervalue(int);
    void jointDataUpdate(float fvalue, int id);
	void updateThresholdSlidervalue(int value);
	void enableThreshold(bool toogle);
	void enableAdaptativeThreshold(bool toogle);

	void changeVisModeForPlane(int);
	void changeSelPointForPlane(int);

protected:
    virtual void keyPressEvent(QKeyEvent* event);

public:
    Ui::MainWindow *ui;

    toolmode toolSelected;
};

#endif // MAINWINDOW_H
