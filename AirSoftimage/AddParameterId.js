function ConfigureDUParamsProperty( oGrp )
{
	if(typeof(oGrp) == "string")
	{
		oGrp = Dictionary.GetObject(oGrp, false);
	}
	
	// Adding Ids and static matrices.
	for( var i = 0; i< oGrp.Members.Count; i++)
	{
		var oObj = oGrp.Members.Item(i);
		LogMessage("\t" + i + " : " + oObj.Name);
		
		var oICEBoneProp = Dictionary.GetObject( oObj + ".ICEBoneParams", false);
		if( !oICEBoneProp)
		{
			oICEBoneProp = oObj.AddCustomProperty("ICEBoneParams", false);
		}
		
		var oParam = oICEBoneProp.Parameters("BoneID");
		if(!oParam)
		{
			oParam = oICEBoneProp.AddParameter3("BoneID", siInt2, -1, -1, 999);	
		}

		oParam.PutValue2(0, i);
		
		var oParam = oICEBoneProp.Parameters("expansion");
		if(!oParam)
		{
			oParam = oICEBoneProp.AddParameter3("expansion", siFloat, 1, 0, 10);	
		}
		
		oParam.PutValue2(0, 1.0);
		
		var oParam = oICEBoneProp.Parameters("propagation");
		if(!oParam)
		{
			oParam = oICEBoneProp.AddParameter3("propagation", siFloat, 1, 0, 10);	
		}
		
		oParam.PutValue2(0, 1.0);
		
		var oobj = oGrp.Members.Item(i);
		if(!Dictionary.GetObject(oObj + ".Static_Kinematic_State_Property", false ) )
		{
			oObj.AddProperty("Static Kinematic State Property" );
		}
	}
			
	SetEnvelopeRefPoses( oGrp );
	
	// Adding parent relations
	for( var i = 0; i< oGrp.Members.Count; i++)
	{
		var oObj = oGrp.Members.Item(i);
		var oICEBoneProp = Dictionary.GetObject( oObj + ".ICEBoneParams", false);
		if( !oICEBoneProp)
		{
			oICEBoneProp = oObj.AddCustomProperty("ICEBoneParams", false);
		}
		
		// Adding parameter
		var oParentParam = oICEBoneProp.Parameters("ParentID");
		if(!oParentParam)
		{
			oParentParam = oICEBoneProp.AddParameter3("ParentID", siInt2, -1, -1, 999);	
		}
		
		// Assining parent relation
		var oObjParent = oObj.Parent;
		if( oGrp.IsMember( oObjParent ) )
		{
			oParentParam.PutValue2(0, GetValue(oObjParent + ".ICEBoneParams.BoneID") );
		}
		else
		{
			// It is a root node.
			oParentParam.PutValue2(0,-1);
		}
	}

}

ConfigureDUParamsProperty("deformers");