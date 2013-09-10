#include "SelectionManager.h"


void selectionManager::drawFunc()
{
/*
    if(dirtyFlag) update();

    if(ctx == CTX_MOVE)
        h.type = axis;
    else if(ctx == CTX_ROTATION)
        h.type = trisphere;
    else if(ctx == CTX_SELECTION)
    {
        return;
    }

    if(selection.size() == 0)
        return;

    h.drawFunc();
*/
}


#include <dataStructures/DataStructures.h>

bool selectionManager::update()
{
	/*
    Point3d selCenter(0,0,0);
    bool loaded = false;
    int elements = 0;

    for(unsigned int i = 0; i< selection.size(); i++)
    {
        if(mode == SM_OBJECT)
        {
            if(!loaded)
            {
                selCenter = ((object*)selection[i])->pos; loaded = true;
            }
            else
                selCenter += ((object*)selection[i])->pos;

            elements++;
        }
        else if(mode == SM_SUBOBJECT)
        {
            if(!loaded)
               { selCenter = ((object*)selection[i])->getSelCenter(); loaded = true;}
            else
                selCenter += ((object*)selection[i])->getSelCenter();

            elements++;
        }
    }

    if(elements > 0)
        selCenter = selCenter / elements;

    dirtyFlag = false;
	*/
    return true;

}
