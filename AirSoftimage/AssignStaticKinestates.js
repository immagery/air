function AssignStaticKinestates (oGrp)
{
	if( typeof( oGrp ) == "string" )
	{
		oGrp = Dictionary.GetObject(oGrp, false);
	}
	
	for( var i= 0; i< oGrp.Members.Count; i++)
	{
		var oObj = oGrp.Members.Item(i);
		if(!Dictionary.GetObject(oObj + ".Static_Kinematic_State_Property", false ) )
		{
			oObj.AddProperty("Static Kinematic State Property" );
		}
	}
	
	SetEnvelopeRefPoses( oGrp );

}

AssignStaticKinestates( "Meshes" );