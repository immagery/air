#ifndef AIR_OUTLINER_H
#define AIR_OUTLINER_H

#include <gui/outliner.h>

using namespace std;

class DefGroup;
class AirOutliner : public outliner
{
public:

    AirOutliner() : outliner() {}
    AirOutliner(scene* scena)  : outliner(scena) {}
	
	virtual void getSceneTree(treeNode* root);

private:
	void getSceneTree(DefGroup* group,treeNode* root);
};

#endif // AIR_OUTLINER_H
