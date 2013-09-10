# the OpenMaya module has many of the general Maya classes
# note that some Maya classes require extra modules to use which is noted in the API docs
import maya.OpenMaya as OpenMaya

def exportMesh(output) :

	# This shows how to use the MSelectionList and MGlobal class
	# Get the selection and create a selection list of all the nodes meshes
	selection = OpenMaya.MSelectionList()
	OpenMaya.MGlobal.getActiveSelectionList( selection );

	# Create an MItSelectionList class to iterate over the selection
	# Use the MFn class to as a filter to filter node types
	iter = OpenMaya.MItSelectionList ( selection, OpenMaya.MFn.kGeometric );

	totalVerts = 0
	totalPoligons = 0
	totalTriangles = 0

	usedVerticesDic = {}

	exportVertices={}
	exportTriangles={}

	# This uses build in functions of the MItSelectionList class to loop through the list of objects
	# Note this is not a basic array you must use its built in functions iterate on its objects
	# Iterate through selection
	while not iter.isDone():

		# Get the selection as an MObject
		mObj = OpenMaya.MObject()
		iter.getDependNode( mObj )

		# This shows how to use the MItMeshPolygon class to work with meshes
		# Create an iterator for the polygons of the mesh
		iterPolys = OpenMaya.MItMeshPolygon( mObj )

		count = 0;

		# Iterate through polys on current mesh
		while not iterPolys.isDone():
			# numero de poligonos
			totalPoligons +=1;

			# Get current polygon triangles
			pntAry = OpenMaya.MPointArray()
			intAry = OpenMaya.MIntArray()
			space = OpenMaya.MSpace.kObject

			# Get the vertices and vertex positions of all the triangles in the current face's triangulation.
			iterPolys.getTriangles(pntAry, intAry, space)

			if not len(intAry) % 3 == 0 :
				raise RuntimeError, 'Hay poligonos con triangulacion defectuosa';

			nSubTriangles = len(intAry) / 3;
			
			for tr in range(nSubTriangles):
				exportTriangles[totalTriangles] = [totalVerts+intAry[tr*3+0], totalVerts+intAry[tr*3+1], totalVerts+intAry[tr*3+2]];
				totalTriangles += 1;		

			for x in range(len(intAry)):
				usedVerticesDic[totalVerts+intAry[x]] = True;

			# numero de triangulos
			# ntri;
			# if iterPolys.hasValidTriangulation():
			#	iterPolys.numTriangles(ntri);
			#	totalTriangles += ntri;

			# Move to next polygon in the mesh list
			iterPolys.next()

		iterVerts = OpenMaya.MItMeshVertex( mObj );

		while not iterVerts.isDone():
			pos = iterVerts.position();
			exportVertices[totalVerts] = [pos[0], pos[1], pos[2]];
			totalVerts += 1;
			iterVerts.next();

		# Move to the next selected node in the list
		iter.next()

	print 'Total Polygons: %d' % totalPoligons
	print 'Total Triangles: %d' % totalTriangles
	print 'Vertices: %d' % totalVerts
	print 'UsedVertices: %d' % len(usedVerticesDic)

	colors = 0;

	output.write("OFF\n");
	output.write("%d %d %d\n" % (totalVerts, totalTriangles, colors));

	for v in range(totalVerts):
		output.write("%f %f %f\n" % (exportVertices[v][0], exportVertices[v][1], exportVertices[v][2]))

	for f in range(totalTriangles):
		output.write("3 %d %d %d\n" % (exportTriangles[f][0], exportTriangles[f][1], exportTriangles[f][2]))