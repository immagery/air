/*

    This file is part of GEAR.

    GEAR is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/lgpl.html>.

    Author:     Jeremie Passerin      geerem@hotmail.com
    Date:       2011 / 03 / 21

*/

function gear_zipper_Update(In_UpdateContext, Out, crv0, crv1)
{
	// Inputs ---------------------------------------------------------
	zip 	= In_UpdateContext.Parameters( "zip" ).value; 
	bias 	= In_UpdateContext.Parameters( "bias" ).value; 
	smooth 	= In_UpdateContext.Parameters( "smooth" ).value; 
	fcurve 	= In_UpdateContext.Parameters( "fcurve" ).value; 
	type 	= In_UpdateContext.Parameters( "type" ).value; 
	
    crv0 = crv0.Value.ActivePrimitive.Geometry;
    ncrv0 = crv0.Curves(0);
    crv1 = crv0.Value.ActivePrimitive.Geometry.Curves(1);
    ncrv1 = crv1.Curves(0);
    
    pnt0_count  = crv0.Points.Count;
    pnt1_count  = crv1.Points.Count;
    
    pnt0_pos = crv0.Points.PositionArray;
    pnt1_pos = crv1.Points.PositionArray;
    
    if(type == 0)
    {
    }
	
	// Process --------------------------------------------------------

	
	// Out ------------------------------------------------------------


	Out.Value.Transform = tOut
}

/*

def gZipperOp_Update(ctxt):

    # Inputs -----------------------------------------------
    OutPort = ctxt.OutputPort

    oCurveGeo0 = ctxt.GetInputValue(0, 0, 0).Geometry
    oNurbsCurve0 = oCurveGeo0.Curves(0)
    oCurveGeo1 = ctxt.GetInputValue(1, 0, 0).Geometry
    oNurbsCurve1 = oCurveGeo1.Curves(0)

    dZip = ctxt.GetParameterValue("Zip")
    dIntersection = ctxt.GetParameterValue("Intersection")
    dSmooth = ctxt.GetParameterValue("Smooth")
    oFCurve = ctxt.GetParameterValue("FCurve")
    iType = ctxt.GetParameterValue("Type")

    iPoints0 = oCurveGeo0.Points.Count
    iPoints1 = oCurveGeo1.Points.Count

    aTup0 = oCurveGeo0.Points.PositionArray
    aTup1 = oCurveGeo1.Points.PositionArray

    if iType == 0:
        aPos0 = [aTup0[j][i] for i in range(len(aTup0[0])) for j in range(len(aTup0))]
        aPos1 = [aTup1[j][i] for i in range(len(aTup1[0])) for j in range(len(aTup1))]
    else:
        dStep = 100  / (iPoints1-1.0)
        a = [oNurbsCurve0.EvaluatePositionFromPercentage(i*dStep)[0].Get2() for i in range(iPoints1)]
        aPos0 = [ a[j][i] for j in range(len(a)) for i in range(len(a[0])) ]

        dStep = 100  / (iPoints0-1.0)
        a = [oNurbsCurve1.EvaluatePositionFromPercentage(i*dStep)[0].Get2() for i in range(iPoints0)]
        aPos1 = [ a[j][i] for j in range(len(a)) for i in range(len(a[0])) ]

    aMidPos = [(aPos0[i]*dIntersection+aPos1[i]*(1-dIntersection)) for i in range(len(aPos0))]

    # Process -----------------------------------------------
    if OutPort.Index == 2: 
        t = aTup0
        p = iPoints0
    else: 
        t = aTup1
        p = iPoints1

    aPos = []
    for i in range(p):

        dStep = 1/(p-1.0)

        v0 = XSIMath.CreateVector3(t[0][i], t[1][i], t[2][i])
        v1 = XSIMath.CreateVector3(aMidPos[i*3+0], aMidPos[i*3+1], aMidPos[i*3+2])

        d = oFCurve.Eval(i/(p-1.0))

        vR = XSIMath.CreateVector3()
        vR.Sub(v1, v0)

        if dZip < d:
            y = 0
        else:
            y = changerange(dZip, d, d+dStep+dSmooth, 0, 1)

        vR.ScaleInPlace(min(1,y))

        aPos.append(v0.X + vR.X)
        aPos.append(v0.Y + vR.Y)
        aPos.append(v0.Z + vR.Z)

    # Output ------------------------------------------------

    Out = ctxt.OutputTarget
    Out.Geometry.Points.PositionArray = aPos

def changerange(x, a, b, c, d):
    return c + ( x - a ) * (( d-c) / (b-a+0.0))
*/