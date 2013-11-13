#ifndef AIR_SEGMENTATION_H
#define AIR_SEGMENTATION_H

#include <DataStructures\AirRig.h>

// Updates all the info for compute the skinning and computes it.
void updateAirSkinning(DefGraph& graph, Modelo& model);

// Segment the model using the air deformers.
void segmentModelFromDeformers(Modelo& model, binding* bd, DefGraph& graph);

// Compute the hierarchical skinning using computed segmentation
void propagateHierarchicalSkinning(Modelo& model, binding* bd);

void createTraductionTable(DefGroup* group, map<int, int>& traductionTable, int idNode, bool onlyRoot = false);

void propagateHierarchicalSkinning(Modelo& model, binding* bd, DefGraph& graph);
void propagateHierarchicalSkinning(Modelo& model, binding* bd, DefGraph& graph, DefGroup& group);

// Compute secondary weights - now is not updating data, it computes what ever it needs.
void computeSecondaryWeights(Modelo& model, binding* bd, DefGraph& graph);

int indexOfNode(int nodeId, vector<DefNode>& nodes);

#endif // AIR_SEGMENTATION_H