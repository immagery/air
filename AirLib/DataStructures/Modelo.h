#ifndef MODEL_H
#define MODEL_H

#include "Geometry.h" // Data Structures based un vcg library
#include "Cage.h"
#include "Deformer.h"
#include "grid3D.h"
#include "SurfaceData.h"

class Modelo : public Geometry
{
public:

    Modelo();
    Modelo(unsigned int id);
    ~Modelo();

    virtual void Clear();

    Cage* currentCage;

    Geometry* currentRender;

    // Cajas para hacer la deformacion
    Cage* modelCage; // Caja envolvente original
    Cage* dynCage; // Caja que puede variar interactivamente
    vector< Cage* > stillCages; // Cajas est‡ticas de posiciones concretas (tests)

    /// Biharmonic Distances matrix (nxn)
    vector< vector< double> > embedding;

    /// Skeletons binded
    vector<binding*> bindings;

	vector<TriangleData> virtualTriangles;

	bool computedBindings;

	vector<int> modelVertexDataPoint;
	vector<int> modelVertexBind;
	vector<int> globalIndirection;

    //grid3d* grid;

    string sModelPath; // Ruta donde est‡ el modelo
    string sModelPrefix; // Prefijo de los ficheros del modelo

    // Plugs de salida
    vector< deformer* > deformers; // Applied deformers

    virtual void drawFunc();

    virtual void select(bool bToogle, unsigned int id);

	void initGrid();

	void setSpotVertex(int i); 
	void setSpotVertexes(vector<int>& indices);
	void addSpotVertex(int i);
	void cleanSpotVertexes();
};


#endif // MODEL_H
