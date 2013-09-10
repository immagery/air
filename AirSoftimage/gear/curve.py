'''

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
    Company:    Studio Nest (TM)
    Date:       2010 / 11 / 15

'''

## @package gear.xsi.curve
# @author Jeremie Passerin
#
# @brief create, merge, split curves...

##########################################################
# GLOBAL
##########################################################
# gear
from gear.xsi import xsi, c, XSIMath, XSIFactory
import gear.xsi.utils as uti

##########################################################
# DRAW
##########################################################
# ========================================================
## Create a curve attached to given centers. One point per center.\n
# Do to so we use a cluster center operator per point. We could use an envelope (method to do so is in the code), but there was a reason I can't remember why it was better to use clustercenter.
# @param parent X3DObject - Parent object.
# @param name String - Name.
# @param centers List of X3DObject or Collection - Object that will drive the curve.
# @param close Boolean - True to close the fcurve.
# @param degree Integer - 1 for linear curve, 3 for Cubic.
# @return NurbCurve - The newly created curve.
def addCnsCurve(parent, name, centers, close=False, degree=1):

    # convert collections to list
    centers = [center for center in centers]

    if degree == 3:
        if len(centers) == 2:
            centers.insert(0, centers[0])
            centers.append(centers[-1])
        elif len(centers) == 3:
            centers.append(centers[-1])

    points = []
    for center in centers:
        points.append(center.Kinematics.Global.Transform.PosX)
        points.append(center.Kinematics.Global.Transform.PosY)
        points.append(center.Kinematics.Global.Transform.PosZ)
        points.append(1)

    curve = parent.AddNurbsCurve(points, None, close, degree, c.siNonUniformParameterization, c.siSINurbs, name)
    crv_geo = curve.ActivePrimitive.Geometry

    for i, center in enumerate(centers):
        cluster = crv_geo.AddCluster( c.siVertexCluster, "center_%s"%i, [i] )
        xsi.ApplyOp( "ClusterCenter", cluster.FullName+";"+center.FullName, 0, 0, None, 2)

#    Here is a method to replace the cluster centers with an envelope
#    envelopeop = curve.ApplyEnvelope(cCenters)
#
#    aWeights = []
#    for i in range(cCenters.Count):
#        for j in range(cCenters.Count):
#             if i == j:
#                 aWeights.append(100)
#             else:
#                 aWeights.append(0)
#
#    envelopeop.Weights.Array = aWeights

    return curve

# ========================================================
## Create a NurbsCurve with a single subcurve.
# @param parent X3DObject - Parent object.
# @param name String - Name.
# @param points List of Double - positions of the curve in a one dimension array [point0X, point0Y, point0Z, 1, point1X, point1Y, point1Z, 1, ...].
# @param close Boolean - True to close the curve.
# @param degree Integer - 1 for linear curve, 3 for Cubic.
# @param t SITransformation - Global transform.
# @param color List of Double - The RGB color of the Null (ie. [1,0,0] for red).
# @return NurbCurve - The newly created curve.
def addCurve(parent, name, points, close=False, degree=1, t=XSIMath.CreateTransform(), color=[0,0,0]):

    curve = parent.AddNurbsCurve(points, None, close, degree, c.siNonUniformParameterization, c.siSINurbs, name)

    uti.setColor(curve, color)
    curve.Kinematics.Global.Transform = t

    return curve

# ========================================================
## Create a NurbsCurve with multiple subcurve.
# @param parent X3DObject - Parent object.
# @param name String - Name.
# @param points List of Double - positions of the curve in a one dimension array [point0X, point0Y, point0Z, 1, point1X, point1Y, point1Z, 1, ...].
# @param ncp List of Double - See XSI SDK Docv for AddNurbsCurveList2.
# @param kn List of Double - See XSI SDK Docv for AddNurbsCurveList2.
# @param nkn List of Double - See XSI SDK Docv for AddNurbsCurveList2.
# @param close List of Boolean - True to close the curve.
# @param degree List of Integer - 1 for linear curve, 3 for Cubic.
# @param t SITransformation - Global transform.
# @param color List of Double - The RGB color of the Null (ie. [1,0,0] for red).
# @return NurbCurve - The newly created curve.
def addCurve2(parent, name, points, ncp=[], kn=[], nkn=[], close=[], degree=[], t=XSIMath.CreateTransform(), color=[0,0,0]):

    pointCount = len(ncp)
    aPar = [c.siNonUniformParameterization for i in range(pointCount)]

    curve = parent.AddNurbsCurveList2(pointCount, points, ncp, kn, nkn, close, degree, aPar, c.siSINurbs, name)

    uti.setColor(curve, color)
    curve.Kinematics.Global.Transform = t

    return curve

# ========================================================
## Create a NurbsCurve with a single subcurve from a list of position.
# @param parent X3DObject - Parent object.
# @param name String - Name.
# @param positions List of SIVector3 - positions of the curve points.
# @param close Boolean - True to close the curve.
# @param degree Integer - 1 for linear curve, 3 for Cubic.
# @param t SITransformation - Global transform.
# @param color List of Double - The RGB color of the object (ie. [1,0,0] for red).
# @return NurbCurve - The newly created curve.
def addCurveFromPos(parent, name, positions, close=False, degree=1, t=XSIMath.CreateTransform(), color=[0,0,0]):

    points = []

    for v in positions:
        points.append(v.X)
        points.append(v.Y)
        points.append(v.sZ)
        points.append(1)

    curve = parent.AddNurbsCurve(points, None, close, degree, c.siNonUniformParameterization, c.siSINurbs, name)

    uti.setColor(curve, color)
    curve.Kinematics.Global.Transform = t

    return curve 
    
##########################################################
# SUBCURVES
##########################################################
# Merge Curves ===========================================
## Merge given curve in one unique curve.
# @param curve List of NurbsCurve - The curves to merge.
# @return NurbsCurve.
def mergeCurves(curves):

    points = []
    ncp = []
    kn = []
    nkn = []
    closed = []
    degree = []

    for curve in curves:

        curve_matrix = curve.Kinematics.Global.Transform.Matrix4

        for nurbscrv in curve.ActivePrimitive.Geometry.Curves:

            ncp.append(nurbscrv.ControlPoints.Count)
            kn.extend(nurbscrv.Knots.Array)
            nkn.append(len(nurbscrv.Knots.Array))
            closed.append(isClosed(nurbscrv))
            degree.append(nurbscrv.Degree)

            for point in nurbscrv.ControlPoints:

                point_pos = point.Position
                point_pos.MulByMatrix4InPlace(curve_matrix)

                points.extend([point_pos.X, point_pos.Y,point_pos.Z, 1])

    if len(ncp) > 1:
        curve = addCurve2(xsi.ActiveSceneRoot, "curve", points, ncp, kn, nkn, closed, degree)
    else:
        curve = addCurve(xsi.ActiveSceneRoot, "curve", points, closed[0], degree[0])

    return curve

# Split Curves ===========================================
## Split the sub curve of given curve.
# @param curve NurbsCurve - The curves to split.
# @return List of NurbsCurve.
def splitCurve(curve):

    t = curve.Kinematics.Global.Transform
    curves = [addCurve(curve.Parent, curve.Name+str(i), nurbscrv.ControlPoints.Array, isClosed(nurbscrv), nurbscrv.Degree, t) for i, nurbscrv in enumerate(curve.ActivePrimitive.Geometry.Curves)]

    return curves

# Is Closed ==============================================
## Return true if the given nurbscurve is closed.
# @param nurbscrv NurbsCurve - The nurbs curves to check.
# @return Boolean.
def isClosed(nurbscrv):
     if nurbscrv.Degree ==  3:
          return not nurbscrv.ControlPoints.Count == (len(nurbscrv.Knots.Array)-2)
     else:
          return not nurbscrv.ControlPoints.Count == len(nurbscrv.Knots.Array)

##########################################################
# OPERATOR
##########################################################
# Apply Curve Resampler Op ===============================
## Resample the curve on itself, code of the operator is in the plugin sn_CurveTools
# @param curve NurbsCurve - The curve to resample.
# @return Operator
def applyCurveResamplerOp(curve):

    op = XSIFactory.CreateObject("gear_CurveResamplerOp")
    op.AddIOPort(curve.ActivePrimitive)
    op.Connect()

    return op
