#ifndef SHADINGNODE_H
#define SHADINGNODE_H
#include <DataStructures/Node.h>

enum shadingMode{T_POINTS = 0, T_LINES, T_POLY, T_XRAY, T_INVISIBLE};

#include <vector>
using namespace std;


class object;
class shadingNode: public node
{
public:
    shadingNode() : node()
    {
        visible = 1;
        selected = 0;
        subObjectmode = 0;
        xray = 0;
        shtype = T_POLY;

        // Color por defecto
        color[0] = 1.0;
        color[1] = 1.0;
        color[2] = 1.0;

        colors.clear();

		m_bDrawPoints = false;
    }

    shadingNode(unsigned int id) : node(id)
    {
        visible = 1;
        selected = 0;
        subObjectmode = 0;
        xray = 0;
        shtype = T_POLY;

        // Color por defecto
        color[0] = 1.0;
        color[1] = 1.0;
        color[2] = 1.0;

        colors.clear();
		m_bDrawPoints = false;
    }

    bool visible;
    bool selected;
    bool subObjectmode;

    bool xray;

    shadingMode shtype;

	bool m_bDrawPoints;

    float color[3];

    vector< vector< float > > colors;

    vector< unsigned int > selectedIds;

	//drawing functions
	bool update(object* obj);
	void beforeDraw(object* obj);
	void afterDraw(object* obj);

	//void drawFunc(object* obj);

};

#endif // SHADINGNODE_H
