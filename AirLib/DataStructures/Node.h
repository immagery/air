#ifndef NODE_H
#define NODE_H

#include <string>

enum nodeTypes {NODE = 0, RENDERNODE_NODE, GRIDRENDERER_NODE, PLANERENDERER_NODE, OBJECT_NODE, DEFORMER_NODE, SKELETON_NODE, JOINT_NODE};

class node
{
    public:
    // Dirty flag management
    bool dirtyFlag;

    unsigned int nodeId;

    std::string sPath; // Node path
    std::string sName; // Node name

    nodeTypes iam;

    node()
    {
        dirtyFlag = false;
        sName.clear();
        sPath.clear();
        nodeId = 0;
        iam = NODE;
    }

    node(unsigned int id)
    {
        dirtyFlag = false;
        sName.clear();
        sPath.clear();

        nodeId = id;
        iam = NODE;
    }

    virtual bool propagateDirtyness()
    {
        dirtyFlag = false;

        return true;
    }

    virtual bool update()
    {
        if(!dirtyFlag)
            return true;
        else
            return true;
    }

};

#endif // NODE_H
