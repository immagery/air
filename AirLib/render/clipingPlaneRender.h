#ifndef CLIPPLANERENDER_H
#define CLIPPLANERENDER_H

#include "DataStructures\DataStructures.h"

#include <globalDefinitions.h>
#include "shadingNode.h"
#include <DataStructures/object.h>
#include <stdio.h>

using namespace vcg;

enum visualizatoinMode {VIS_WEIGHTS_ONPLANE = 0, VIS_DISTANCES};

#define XY_PLANE false
#define YZ_PLANE true

class clipingPlaneRenderer : public shadingNode
{
public:

	clipingPlaneRenderer(int id):shadingNode(id)
    {
		init();
	}
    // Constructors
    clipingPlaneRenderer():shadingNode()
    {
        init();
    }

    void init()
    {
        //grid = NULL;
        dirtyFlag = 1;
        //dirtyXZ = 1;
        //dirtyXY = 1;
        //m_bShowAllGrid = false;

        //XYValue = 0;
        //XZValue = 0;

        //m_iCurrentSliceXZ = 0;
        //m_iCurrentSliceXY = 0;

        //desiredVertex = 0;

        //Initialized = 0;

        //iVisMode = VIS_LABELS;

        iam = PLANERENDERER_NODE;

        //sliceValuesXY.clear();
        //sliceValuesXZ.clear();

        //m_bBorders = 0;

        //m_bShow_interior = 0;
        //m_bShow_exterior = 0;
        //m_bShow_boundary = 0;

        //bshowSegmentation = 0;
        //bShowWeights = 0;

        //segmentationIndicesToShow.clear(); // Segmentation labels to show

        //model = NULL;
		posUpdated = false;

		orientation = XY_PLANE;
		subdivisions = 0;

		minPoint = Point3d(0,0,0);
		minPoint = Point3d(0,0,0);

		weights.clear();

		selectedPoint = 0;

		// For drawing optimizing
		points.clear();
		colors.clear();

		vmode = VIS_WEIGHTS_ONPLANE;

		plane = new object();
    }

    void clear()
    {
        // TODEBUG
        //m_iCurrentSliceXZ = m_iCurrentSliceXY = 0;
    }

    ~clipingPlaneRenderer()
    {

    }

    virtual bool propagateDirtyness()
    {
        // TODO
        dirtyFlag = 1;
        //dirtyXY = 1;
        //dirtyXZ = 1;

        return true;
    }

    // asignación del grid a renderizar
    //void setGrid(grid3d* _grid) {grid = _grid;}

    // currentSlice variables and dirt the flag
    void setSliceXY(int sliceXY)
    {
        //if(m_iCurrentSliceXY != sliceXY)
        //{
        //    m_iCurrentSliceXY = sliceXY;
        //    dirtyXY = 1;
        //    dirtyFlag = 1;
        //}
    }

    void setSliceXZ(int sliceXZ)
    {
        //if(m_iCurrentSliceXZ != sliceXZ)
        //{
        //    m_iCurrentSliceXZ = sliceXZ;
        //    dirtyXZ = 1;
        //    dirtyFlag = 1;
        //}
    }

   // void updateSlices(); // currentSlice variables and dirty the flag
//	void updateSlicesForSegmentation(int maxRange);
	//void updateSlicesForVolumeLabels(int maxRange);
	//void updateSlicesForWeights(float maxRange, int searchedIndex);

    virtual bool update(object* obj);
    virtual void beforeDraw(object* obj);
	virtual void drawFunc(object* obj);

    void init(int numLabels);

	bool orientation;
	double position;
	int subdivisions;

	int selectedPoint;

	Point3d minPoint;
	Point3d maxPoint;

	vector< vector<double> > weights;

	// For drawing optimizing
	vector<vcg::Point3d> points;
	vector<vcg::Point3f> colors;
	bool posUpdated;

	visualizatoinMode vmode;

	object* plane;

};


#endif // CLIPPLANERENDER_H
