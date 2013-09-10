#ifndef GRIDRENDER_H
#define GRIDRENDER_H

#include <globalDefinitions.h>
#include <DataStructures/grid3D.h>

#include "shadingNode.h"

#include <stdio.h>

class Modelo;
class skeleton;

void drawPointLocator(Point3d pt, float size, bool spot);

class gridRenderer : public shadingNode
{
public:

    // Constructors
    gridRenderer():shadingNode()
    {
        init();
    }

    void init()
    {
        //grid = NULL;
        dirtyFlag = 1;
        dirtyXZ = 1;
        dirtyXY = 1;
        m_bShowAllGrid = false;

        XYValue = 0;
        XZValue = 0;

        m_iCurrentSliceXZ = 0;
        m_iCurrentSliceXY = 0;

        desiredVertex = 0;

        Initialized = 0;

        iVisMode = VIS_LABELS;

        iam = GRIDRENDERER_NODE;

        sliceValuesXY.clear();
        sliceValuesXZ.clear();

        m_bBorders = 0;

        m_bShow_interior = 0;
        m_bShow_exterior = 0;
        m_bShow_boundary = 0;

        bshowSegmentation = 0;
        bShowWeights = 0;

        segmentationIndicesToShow.clear(); // Segmentation labels to show

        model = NULL;
    }

    void clear()
    {
        // TODEBUG
        m_iCurrentSliceXZ = m_iCurrentSliceXY = 0;
    }

    ~gridRenderer()
    {

    }

    virtual bool propagateDirtyness()
    {
        // TODO
        dirtyFlag = 1;
        dirtyXY = 1;
        dirtyXZ = 1;

        return true;
    }

    // asignación del grid a renderizar
    //void setGrid(grid3d* _grid) {grid = _grid;}

    // currentSlice variables and dirt the flag
    void setSliceXY(int sliceXY)
    {
        if(m_iCurrentSliceXY != sliceXY)
        {
            m_iCurrentSliceXY = sliceXY;
            dirtyXY = 1;
            dirtyFlag = 1;
        }
    }

    void setSliceXZ(int sliceXZ)
    {
        if(m_iCurrentSliceXZ != sliceXZ)
        {
            m_iCurrentSliceXZ = sliceXZ;
            dirtyXZ = 1;
            dirtyFlag = 1;
        }
    }

    void updateSlices(); // currentSlice variables and dirty the flag
	void updateSlicesForSegmentation(int maxRange);
	void updateSlicesForVolumeLabels(int maxRange);
	void updateSlicesForWeights(float maxRange, int searchedIndex);

    virtual bool update(object* obj);
    virtual void beforeDraw(object* obj);
	virtual void drawFunc(object* obj);

    void init(int numLabels);

    bool m_bShow_interior; // Enable showing interior voxels
    bool m_bShow_exterior; // Enable showing exterior voxels
    bool m_bShow_boundary; // Enable showing boundary voxels

	bool m_bShowAllGrid; // Enable showing all cubes of the grid restricted by segmentationIndicesToShow...

    bool m_bBorders; // Show frontiers.

    int iVisMode;

    bool bshowSegmentation; // Enable showing segmentation labels
    bool bShowWeights;      // Enable showing weights

    int m_iCurrentSliceXZ; // Current slice in XZ plane in the slider
    int m_iCurrentSliceXY; // Current slice in XY plane in the slider

    int XYValue; // Current slice in XZ plane in the grid
    int XZValue; // Current slice in XY plane in the grid

    bool dirtyXZ;
    bool dirtyXY;

    bool Initialized;

    vector<bool> segmentationIndicesToShow; // Segmentation labels to show
    vector< vector<Point3f> > sliceValuesXZ; // Cached values for coloring plane XZ
    vector< vector<Point3f> > sliceValuesXY; // Cached values for coloring plane XZ

    int desiredVertex; // Vertex for showing weights

    Modelo* model;
    grid3d* grid;
};


#endif // GRIDRENDER_H
