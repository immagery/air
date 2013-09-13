#ifndef AIRICENODE_H
#define AIRICENODE_H

// AirICENode Plugin
// Initial code generated by Softimage SDK Wizard
// Executed Tue Jan 8 11:12:54 UTC+0100 2013 by CHUS
// 
// 
// Tip: You need to compile the generated code before you can load the plug-in.
// After you compile the plug-in, you can load it by clicking Update All in the Plugin Manager.

#include <xsi_string.h>

#include <xsi_menu.h>
#include <xsi_project.h>
#include <xsi_pluginregistrar.h>
#include <xsi_argument.h>
#include <xsi_selection.h>

#include <xsi_customoperator.h>
#include <xsi_operatorcontext.h>
#include <xsi_outputport.h>
#include <xsi_portgroup.h>
#include <xsi_ppglayout.h>
#include <xsi_ppgeventcontext.h>
#include <xsi_ppglayout.h>

#include <xsi_application.h>
#include <xsi_factory.h>
#include <xsi_context.h>
#include <xsi_status.h>

#include <xsi_selection.h>
#include <xsi_command.h>

#include <xsi_kinematics.h>
#include <xsi_kinematicstate.h>

#include <xsi_scene.h>
#include <xsi_sceneitem.h>

//CHAINS & BONES
#include <xsi_chainbone.h>
#include <xsi_chainroot.h>
#include <xsi_envelope.h>

//MODEL
#include <xsi_primitive.h>
#include <xsi_geometry.h>
#include <xsi_facet.h>
#include <xsi_point.h>
#include <xsi_x3dobject.h>
#include <xsi_model.h>
#include <xsi_null.h>
#include <xsi_group.h>
#include <xsi_dictionary.h>

//MATHS
#include <xsi_matrix4.h>
#include <xsi_matrix4f.h>
#include <xsi_vector2f.h>
#include <xsi_vector3f.h>
#include <xsi_vector4f.h>
#include <xsi_matrix3f.h>
#include <xsi_math.h>

// C++ STD
#include <iostream>
#include <fstream>
#include <direct.h>

#include <xsi_icenodecontext.h>
#include <xsi_icenodedef.h>
#include <xsi_longarray.h>
#include <xsi_doublearray.h>
#include <xsi_rotationf.h>
#include <xsi_quaternionf.h>
#include <xsi_color4f.h>
#include <xsi_shape.h>
#include <xsi_icegeometry.h>
#include <xsi_iceportstate.h>
#include <xsi_indexset.h>
#include <xsi_dataarray.h>
#include <xsi_dataarray2D.h>

#include <DataStructures\Scene.h>
#include <DataStructures\InteriorDistancesData.h>
#include <render\gridRender.h>

// For storing CSampleData user data objects
#include <vector>

// Simple user data struct
struct CSampleData {
	CSampleData() : nLen(0), pBuffer(NULL) {}

	LONG nLen;
	XSI::MATH::CVector3f* pBuffer;
};

// Defines port, group and map identifiers used for registering the ICENode
enum IDs
{
	ID_IN_embeddingFileName = 1,
	ID_IN_rigging_animation = 2,

	ID_IN_NodePositions = 3,
	ID_IN_NodeIds = 4,

	ID_IN_Model = 5,
	ID_IN_DefParentIds = 6,

	ID_IN_inWeights = 7,
	ID_IN_pointPositions = 8,
	ID_IN_defIdsCount = 9,
	ID_IN_pointsCount = 10,

	ID_IN_sceneScale = 11,
	ID_IN_globalSmoothing = 12,
	ID_IN_expansionFactors = 13,
	ID_IN_propagationFactors = 14,

	ID_IN_rigging_evaluate= 15,

	ID_G_100 = 100,
	ID_G_200 = 200,
	ID_G_101 = 101,
	ID_G_102 = 102,
	ID_G_103 = 103,

	ID_OUT_outWeights = 50,
	ID_OUT_weightsPerPoint = 51,

	ID_TYPE_POINTS = 500,
	ID_STRUCT_POINTS,
	ID_MAP_POINTS,
	ID_TYPE_CNS = 400,
	ID_STRUCT_CNS,
	ID_CTXT_CNS,
	ID_UNDEF = ULONG_MAX
};

using namespace XSI; 

XSI::CStatus RegisterAirICENode( XSI::PluginRegistrar& in_reg );
XSI::CStatus RegisterAirWeightsContextualization( XSI::PluginRegistrar& in_reg );

SICALLBACK AirICENode_Evaluate( ICENodeContext& in_ctxt );
SICALLBACK AirICENode_BeginEvaluate( ICENodeContext& in_ctxt );
SICALLBACK AirICENode_EndEvaluate( ICENodeContext& in_ctxt );
SICALLBACK AirICENode_Init( CRef& in_ctxt );
SICALLBACK AirICENode_Term( CRef& in_ctxt );

void buildSkeleton(joint* jt, ChainBone& bone);
void buildSceneSkeletons(scene* escena);

void readBone(skeleton* skt, joint* root, FILE* fout, scene* escena, Group& grp);
void readBone(skeleton* skt, joint* root, FILE* fout, scene* escena);

void readSkeleton(string fileName, scene* escena, OperatorContext& ctxt);
void readSkeleton(string fileName, scene* escena);

bool LoadSoftImageModel(XSI::Geometry& geom_soft, scene* escena);
bool LoadICEModel(XSI::CICEGeometry& geom_soft, Modelo* m_air, scene* escena);

//bool LoadDataFromDisc(scene* escena, CString sFileName, OperatorContext& ctxt);
bool LoadDataFromDiscICE(scene* escena, CString sFileName);


#endif // AIRICENODE_H
