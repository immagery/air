#ifndef GLWIDGET_H
#define GLWIDGET_H

//Std libraries
#include <vector>

#include <QtOpenGL/QGLWidget>
#include <QtGui/QKeyEvent>
#include <QtGui/QMouseEvent>
#include <QGLViewer/qglviewer.h>

#include "QGLViewer/constraint.h"

#include "DataStructures/DataStructures.h"

#include "SelectionManager.h"

#include "DrawObject.h"

#define INFO_STRING "Info > %1"

#define BUFFER_OFFSET(i) ((char *)NULL + (i))

class MainWindow;
class Object;
class Cage;
class scene;
class skeleton;
class joint;
class gridRenderer;
class Modelo;
class grid3d;

using namespace std;

#define DEBUG_MODE 0


enum viewingModes { DynCage_Mode = 0, Cages_Mode, BHD_Mode, SimpleModel_Mode};

enum shadingModes { SH_MODE_FLAT = 0, SH_MODE_SMOOTH};

class GLWidget : public QGLViewer
{
    Q_OBJECT

public:
    GLWidget(QWidget * parent = 0, const QGLWidget * shareWidget = 0, Qt::WindowFlags flags = 0);
    ~GLWidget();

    QSize minimumSizeHint() const { return QSize(50, 50); }
    QSize sizeHint() const { return QSize(400, 400); }

    virtual void init();
    virtual void animate();

    // LOADING
    void readModel(string fileName, string name, string path);
    void readScene(string fileName, string name, string path);
    void readDistances(QString fileName);

    void readBone(skeleton *skt, joint *root, FILE *fout);
    void readSkeleton(string fileName);
	void readSkeletons(string fileName, vector<skeleton*>& skts);

	float calculateDistancesForISOLines(grid3d* grid, vector<double>&  embeddedPoint);

    //void readCage(QString fileName, Modelo& m_);

    // DRAWING FUNCTIONS
    void ReBuildScene();
    void drawSceneGrid(int lines, double width);
    void drawCube(Point3d o, double cellSize, Point3f color, bool blend = false);
    void initScene();

    //SELECTION
    void selectElements(vector<unsigned int > lst);

    void testScene();

    void drawColored(QVector<QColor> colors);


    // VARIABLES
    vcg::Point3d m_ptCenter; // centro de la escena
    float m_sceneMin[3];
    float m_sceneMax[3];

    float m_sceneRadius;

    viewingModes viewingMode;
    shadingModes ShadingModeFlag;

    void cleanShadingVariables();
    void toogleModelToXRay();
    void toogleModelToLines();
    void toogleVisibility(bool toogle);
    void changeVisualizationMode(int);
	void updateGridVisualization();
    void toogleToShowSegmentation(bool toogle);

    void changeSmoothPropagationDistanceRatio(float smoothRatioValue);
	void cleanWeights(gridRenderer* grRend);
    void changeExpansionFromSelectedJoint(float expValue);

    // State flags
    bool loadedModel;
    bool loadedCages;

    bool colorsLoaded;
    bool colorLayers;

    int colorLayerIdx;

    bool firstInit; // Flag por si es necesario hacer refresh
    bool refreshFlag;
    bool firstDrawing;

    bool bGCcomputed;
    bool bHComputed;
    bool bMVCComputed;

	Point3d interiorPoint;

    // Primitives
    MyMesh simpleCube;
    MyMesh littleCube;

    /*
    MyMesh modeloOriginal;
    MyMesh newModeloGC;
    MyMesh newModeloHC;

    MyMesh cage;
    MyMesh newCage;

    QString sModelPath;
    QString sModelPrefix;
    */

    //vector< object*> modelos; // Todos los objetos que tendremos en la escena
    scene* escena; // Escena con jerarquía

    selectionManager selMgr;

    bool drawCage;
    bool shadeCoordInfluence;
    bool showDeformedModel;

    unsigned int influenceDrawingIdx;

    //std::vector< std::vector<float> > PerVertGC;
    //std::vector< std::vector<float> > PerFaceGC;
    //std::vector< std::vector<float> > PerVertHC;

    vector<float> selectionValuesForColorGC;
    vector<float> selectionValuesForColorHC;
    vector<float> selectionValuesForColorMVC;

    vcg::Point2f maxMinGC;
    vcg::Point2f maxMinHC;
    vcg::Point2f maxMinMVC;

    vector< vector<QColor> > vertexColorLayers;

    vector<QColor> vertexColors;

    vector< vector<double> > BHD_distancias;
    vector< int > BHD_indices;

    bool stillCageAbled;
    int stillCageSelected;

	vector<joint*> CurrentProcessJoints;
	gridRenderer* currentProcessGrid;

    QString sPathGlobal;

    bool updateInfo();

    MainWindow *parent;
protected:
    virtual void drawWithNames();
    virtual void postSelection(const QPoint& point);
    virtual void draw();

    void drawModel();
    void drawWithDistances();
    void drawWithCages();

    virtual void endSelection(const QPoint&);

    // Mouse events functions
    virtual void mousePressEvent(QMouseEvent *e);
    virtual void mouseMoveEvent(QMouseEvent *e);
    virtual void mouseReleaseEvent(QMouseEvent *e);


public slots:

    // GENERAL
    void ChangeSliceXZ(int slice);
    void ChangeSliceXY(int slice);

    void paintModelWithGrid();
	void paintModelWithData();
	void paintPlaneWithData(bool compute = false);
	void setPlaneData(bool drawPlane, int pointPos, int mode, float sliderPos, int orient);

    bool readNodes(vector< string >& nodeNames, vector< Point3d >& nodePoints, QString sFile);
    bool readPoints(vector< Point3d >& points, QString sFile);

    void exportWeightsToMaya();

    void computeProcess();
	void doTests(string fileName, string name, string path);
	void getStatisticsFromData(string fileName, string name, string path);

    void VoxelizeModel(Modelo* m, bool onlyBorders = true);

	void setPlanePosition(float x, float y, float z)
	{
	
	}

	void setThreshold(double value);

    //void ComputeSkining(Modelo* m, gridRenderer* grRend);
    //void updateSkinning(gridRenderer* grRend);

    //void updateSkinningWithHierarchy(gridRenderer* grRend);

	void nextProcessStep();
	void allNextProcessSteps();

	//void PropagateFromSegment(gridRenderer* grRend, int frontId);
	//void initDomainForId(gridRenderer* grRend, int fatherId); // Duplicated -> TODELETE
	//void initGridForHierarchicalskinning(gridRenderer* grRend, int domainId_init);
	//void NormalizeWeights(gridRenderer* grRend, vector<int>& frontIds);

	//void computeHierarchicalSkinning(gridRenderer* grRend); // first Step
    //void computeHierarchicalSkinning(gridRenderer* grRend, joint* jt); // recursive computation.

    //void createTraductionTable(joint* jt, map<int, int> &traductionTable, int idNode);

    void PropFunctionConf();
	
	void paintGrid(gridRenderer* grRend);

    void ChangeViewingMode(int);

    // CAGES
    bool processGreenCoordinates();
    bool processHarmonicCoordinates();
    bool processMeanValueCoordinates();
    bool processAllCoords();
    bool deformMesh();
    void loadSelectableVertex(Cage* cage /* MyMesh& cage*/);
    void showDeformedModelSlot();
    void showHCoordinatesSlot();
    void active_GC_vs_HC(int tab);
    void ChangeStillCage(int id);

    void updateGridRender();

    // Biharmonic Distances
    void UpdateVertexSource(int id); // Updates the source for harmonic distances testing
    void importSegmentation(QString fileName);
    void updateColorLayersWithSegmentation(int maxIdx);

    void setContextMode(contextMode ctx);

    //void loadSelectVertexCombo(MyMesh& cage);
    //void changeVertexSelection(int id);

signals:
    void updateSceneView();
    void jointDataShow(float, int);

private :
  void startManipulation();
  void drawSelectionRectangle() const;
  void addIdToSelection(int id);
  void removeIdFromSelection(int id);

  // Current rectangular selection
  QRect rectangle_;

  // Different selection modes
  enum SelectionMode { NONE, ADD, REMOVE };
  SelectionMode selectionMode_;

  QList<DrawObject*> objects_;
  QList<int> selection_;

};
#endif
