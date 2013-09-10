import maya.cmds as cmds
from math import *
import tkFileDialog

def appendUnique(inVec, elem):
	counter = inVec.count(elem);
	if counter == 0:
		inVec.append(elem);
	return inVec;

def extendUnique(inVec1, inVec2):
	for elem in range(len(inVec2)):
		inVec1 = appendUnique(inVec1, inVec2[elem]);
	return inVec1;

def removeWithVector(inVec1, inVec2):
	for elem in range(len(inVec2)):
		if inVec1.count(inVec2[elem])>0:
			inVec1.remove(inVec2[elem]);

	return inVect1;

def suma(inVec1):
	sumV = 0;
	for x in inVec1:
		sumV += float(x);

	return sumV;



def readWeightsFile(filename = ""):

	nodesNames = []
	pointNames = []
	weights = []

	if not filename :
		return;

	inputFile = open(filename, "r");

	# Firts: we read the number of procesed nodes.
	inputFile.readline();
	numNodes = int(inputFile.readline());

	# Second: number of vertices 
	inputFile.readline(); inputFile.readline(); 
	numVerts = int(inputFile.readline());

	# Third: node names
	inputFile.readline(); inputFile.readline();
	nodeNamesString = inputFile.readline();
	nodeNames = nodeNamesString.split()

	# Fourth: node positions
	#inputFile.readline();
	#nodePointsString = inputFile.readline();
	#nodePointsStringAux = nodePointsString.split("; ");

	# Fifth: weight values
	inputFile.readline(); inputFile.readline();

	print "Empezado la lectura:";
	print "numNodes:%d, numVerts:%d, nodeNames:" %(numNodes, numVerts); 
	print nodeNames;

	for pt in range(numVerts):

		vertWeightsString = inputFile.readline();
		vertWeightsStringAux = vertWeightsString.split("; "); # Quitamos la def del punto
		pointNames = vertWeightsStringAux[0];
		auxWeights = vertWeightsStringAux[1].split();
		weights.append(auxWeights);		 # Obtenemos los pesos.

		if not len(weights[pt]) == numNodes:
			raise RuntimeError, "Hay un problema de lectura de los pesos, no hay los que deberia haber, linea: %d." % pt;

	inputFile.close();



	return 	(nodeNames, pointNames, weights);



# funcion principal
# Carga los pesos computados en los modelos selecionados y sus esqueletos asociados.
def importComputedWeights(filename=""):

	print "-" * 50;
	print " "*15 + "IMPORTACION DE PESOS"
	print "-" * 50;

	if filename == "":
		filename = tkFileDialog.askopenfilename(initialdir="D:\\phd_dev\\Data\\models\\");

	# seleccionamos todos los objetos.
	objsSel = cmds.ls( selection = True, long = True, flatten = True, type = "transform");
	objsSel.extend( cmds.ls( selection = True, long = True, flatten = True, type = "shape"));

	#print "objsSel:%d" % len(objsSel); print objsSel;

	# cogemos todos los puntos.
	pointsSel = cmds.ls( selection = True, long = True, flatten = True, type = "float3");
	pointsSel.extend( cmds.ls( selection = True, long = True, flatten = True, type = "double3"));

	#print "pointsSel:%d" % len(pointsSel); print pointsSel;

	if len(objsSel) < 1 | len(pointsSel) < 1:
		print "The selection does not contain any valid object(s). Accepted object types are:\n";
		print "Transforms, Shapes and Control Points (Vertices or CV:s).\n";
		raise RuntimeError, "No valid object(s) selected. See script editor for details.";

	# We take shape and transformation nodes form de selection
	objectsSel = cmds.ls( objsSel, long = True, type="shape");

	#print "objectsSel:"; print objectsSel;

	shapesFromSel = cmds.listRelatives( objsSel, allDescendents = True, fullPath = True, type = "shape")

	#print "shapesFromSel:"; print shapesFromSel;

	selShapes = extendUnique( objectsSel, shapesFromSel)

	#print "selShapes:"; print selShapes;

	selTransf = cmds.listRelatives( selShapes, parent = True, fullPath = True )

	#print "selTransf:"; print selTransf;

	if selTransf:
		validSel = selTransf;
		validSel.extend(pointsSel);
	else:
		validSel = pointsSel;

	#print "validSel:"; print validSel; 

	# Also, get the Skin Clusters connected to the selection and
	# remove any object from the lists that's not connected to a Skin Cluster
	skinClusterList = []
	remove = []

	for item in validSel:
		if not cmds.objectType(item, isType = "transform"): # it is a point
			pointparts = item.split("\.")
			for transf in selTransf:
				if transf == pointparts[0]:
					remove.append(item);

		itemHist = cmds.listHistory(item)
		itemSkCl = cmds.ls(itemHist, type = "skinCluster");

		if len(itemSkCl) == 0:
			remove.append(item);

		for SkCl in itemSkCl:
			appendUnique(skinClusterList, SkCl);

	#print "validSel:"; print validSel; 
	#print "remove:"; print remove; 
	#print "skinClusterList:"; print skinClusterList;

	if len(remove) > 0:
		validSel = removeWithVector(validSel, remove);

	if len(skinClusterList) == 0:
		raise RuntimeError, "No skin clusters connected to selection";


	# Get the deformer sets connected to the Skin Clusters in the selection
	# and place them in an array so that each set's index is the same as the
	# corresponding Skin Cluster's index.

	allSets = cmds.listSets(type = 2);

	#print "allSets:"; print allSets; 

	pointsToDo = 0;

	setList = []
	for _set in allSets:
		ConnectedSCs = cmds.listConnections(_set, type="skinCluster");

		#print "ConnectedSCs:"; print ConnectedSCs; 

		numCSC = 0;
		setIndex = 0;
		
		if ConnectedSCs:
			for CSC in ConnectedSCs:
				for SC in range(len(skinClusterList)):
					if CSC == skinClusterList[SC]:
						setIndex = SC;
						numCSC += 1;

		if numCSC > 0:
			if numCSC > 1:
				raise RuntimeError, "More than one selected Skin Cluster connected to %s.\n" % _set;
			else:
				setList.append(_set);
				allMemb = cmds.getAttr(_set, size = True );
				pointsToDo += len(allMemb);

	#print "Estamos por aqui"; 

	# pasamos a la lectura del fichero.
	(nodesNames, pointNames, weights) = readWeightsFile(filename)

	#print "Hemos leido el fichero"; 
	#print "Node names: "; print nodesNames; 
	#print "pointNames: "; print pointNames; 

	for setN in range(len(setList)):
		setMembers = cmds.sets( setList[setN], query= True);

		#print "setMembers:"; print setMembers; 

		setPoints = cmds.ls( setMembers, flatten = True, long = True)

		#print "setPoints:"; print setPoints;

		setInflTransF = cmds.skinCluster(skinClusterList[setN], query=True, influence= True)

		#print "setInflTransF:"; print setInflTransF;

		cmds.setAttr( skinClusterList[setN] + ".maintainMaxInfluences" ,0);
		cmds.setAttr( skinClusterList[setN] + ".normalizeWeights" ,0);

		#Quito los nodos repetidos.
		#for tr1 in range(len(setInflTransF)):
		#	if nodesNames.count(setInflTransF[len(setInflTransF)-tr1-1]) > 0:
		#		setInflTransF.remove(len(setInflTransF)-tr1-1);

		for setPtIdx in range(len(setPoints)):

			# Eliminamos toda influencia anterior
			#for tr1 in range(len(setInflTransF)):
			#	cmds.skinPercent( skinClusterList[setN], setPoints[setPtIdx], transformValue = [setInflTransF[tr1], 0.0]);
			tempWeights = [];
			# Asignamos las nuevas influencias
			for tr in range(len(nodesNames)):
				tempWeights.append([nodesNames[tr], float(weights[setPtIdx][tr])]);

			#cmds.skinPercent( skinClusterList[setN], setPoints[setPtIdx], transformValue = [nodesNames[tr], float(weights[setPtIdx][tr])]);
			cmds.skinPercent( skinClusterList[setN], setPoints[setPtIdx], transformValue = tempWeights);

		cmds.setAttr( skinClusterList[setN] + ".normalizeWeights" ,1);


	print "-" * 50;
	print " " * 15 + "FIN DE IMPORTACION"
	print "-" * 50;