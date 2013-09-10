#ifndef SELECTIONMANAGER_H
#define SELECTIONMANAGER_H

#include <vector>
//#include "Handle.h"
#include "DataStructures/Node.h"

class object;

// trabajo en objeto o subobjeto.
enum selectionMode { SM_SUBOBJECT = 0, SM_OBJECT};
enum contextMode { CTX_SELECTION = 0, CTX_MOVE, CTX_ROTATION}; // contexto de herramienta

using namespace std;
class selectionManager : public node
{
public:

    selectionMode mode;
    contextMode ctx;

    vector< object* > selection;

    selectionManager() : node()
    {
       mode = SM_OBJECT;
       ctx = CTX_SELECTION;
    }

    virtual void drawFunc();
    virtual bool update();
};

#endif // SELECTIONMANAGER_H
