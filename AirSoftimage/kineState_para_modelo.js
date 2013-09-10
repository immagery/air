oObj = Dictionary.GetObject("Mesh", false);
if(!Dictionary.GetObject(oObj + ".Static_Kinematic_State_Property", false ) )
{
	oObj.AddProperty("Static Kinematic State Property" );
}
	SetEnvelopeRefPoses( oObj );