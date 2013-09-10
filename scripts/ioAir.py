import maya.cmds as cmds
from euclid import *
from math import *
import OFFexport
import tkFileDialog

print "-" * 30;
print "CARGANDO SCRIPTS DEL PAQUETE AIR";
print "-" * 30;

def subdivideSkeleton( rootJoints = None , desiredSeparation = 1) :

	print "-" * 30;
	print " " * 5 + "SCRIPT DE SUBDIVISION DE ESQUELETOS";
	print "-" * 30;

	if rootJoints == None:
		rootJoints = cmds.ls(sl=True, type='transform');

	if rootJoints == None:
		raise RuntimeError, 'Select a root node to process';

	counter = 0;
	for root in rootJoints:
		counter +=  subdivideBone( root, desiredSeparation );


	print "-" * 30;
	print " " * 5 + "SUBDIVISION REALIZADA";
	print "-" * 30;



def subdivideBone( rootJoint , desiredSeparation = 1) :
		
	children = cmds.listRelatives(rootJoint, children=True, type='transform', fullPath=True);

	# fijamos la seleccion actual como el joint padre
	cmds.select(clear = True);
	cmds.select(rootJoint);

	# obtenemos el nombre que usaremos para los nodos nuevos.
	totalCount = 0;
	rootJointName = cmds.joint( rootJoint, query = True, name = True );
	nodeNames = rootJointName + "_WNode%03d";

	print "procesando root: %s" % rootJointName;

	# obtenemos la posicion en el mundo.
	pos = cmds.xform( rootJoint, query = True, worldSpace = True,  translation = True);
	
	# creamos el nodo que corresponde a este hueso
	count = 0;
	cmds.joint( position = pos, name= nodeNames % count, radius = 0.2);
	# cmds.select(clear = True);
	count = count +1;

	# recorremos los hijos para crear nodos hasta el joint hijo
	boneCount = 0;
	if children:
		for child in children:

			totalNodes= 0;

			childPos = cmds.xform( child, query = True, worldSpace = True,  translation = True);

			v1 = Vector3(pos[0], pos[1], pos[2]);
			v2 = Vector3(childPos[0], childPos[1], childPos[2]);

			boneLength = abs(v1-v2);

			print "Longitud del hueso %d: %f" % (boneCount, boneLength);

			if boneLength == 0:
				print "Que curioso.";
				continue;

			minDivisions = floor(boneLength/desiredSeparation);
			newSeparation = boneLength;

			# print "minDivisions: %d" % minDivisions;

			if minDivisions > 1:
				newSeparation = boneLength / minDivisions;

				# print "newSeparation: %f" % newSeparation;

				#if cmds.addAttr(child,  query = True, longName="DefPropagation", attributeType= True, exists = True ):
				cmds.addAttr(child, longName="DefPropagation",  attributeType= "double" , defaultValue =  1);
				#else:
				#	cmds.setAttr(child+".DefPropagation" , 1);

				nodeCounter = 0;
				for idNode in range(minDivisions-1):

					newLength = newSeparation*(idNode+1);

					newPos = pos + (v2-v1)* (newLength/boneLength);

					# print "newPos: %f %f %f" % (newPos[0],newPos[1],newPos[2]);

					name = nodeNames % count
					print "procesando hijo: %s" % name;
  
					newJoint = cmds.joint( position = newPos, name= nodeNames % count, radius = 0.2);
					
				#	if cmds.addAttr(newJoint, query = True, longName="relative", attributeType= "double", exists = True ):
					cmds.addAttr(newJoint, longName="relative",  attributeType= "double", defaultValue =  newLength/boneLength);
				#	else:
				#		cmds.setAttr(newJoint+".relative" , newLength/boneLength);

					# creamos las conexiones para el comportamiento de deformacion que queramos
					multNode = cmds.shadingNode('multiplyDivide', asUtility=True)
					cmds.connectAttr(child+".rotateX", multNode+".input1X", force=True);
					cmds.connectAttr(newJoint+".relative", multNode+".input2X", force=True);
					cmds.connectAttr(multNode+".outputX", newJoint+".rotateX", force=True);

					count = count +1;
					
					cmds.select(clear = True);
					cmds.select(newJoint);

					nodeCounter = nodeCounter+1;

				# print "Hemos anadido %d nodos" % nodeCounter;

			totalCount += subdivideBone(child, desiredSeparation);
			cmds.select(clear = True);
			cmds.select(rootJoint);

			boneCount = boneCount+1;

	# print "Hemos recorrido %d huesos" % boneCount;	


	return totalCount;


# exportar los datos de un nodo.
def exportNodeData( jointName, jointNode, output) :

	pos = cmds.xform( jointNode, query = True, worldSpace = True,  translation = True);
	output.write("%s %f %f %f \n" % (jointName, pos[0], pos[1], pos[2]));

# exportar los nodos de un hueso y sus hijos sucesivamente
def exportBoneNodes( rootJoint , output, writing) :

	children = cmds.listRelatives(rootJoint, children=True, type='transform', fullPath=True);

	counter = 0;

	if children:
		for child in children:
			jointName = cmds.joint( child, query = True, name = True );
			# print "Nombre: %s" % jointName;

			if jointName.count("_WNode") > 0 :
				counter +=1;
				if writing:
					exportNodeData(jointName, child, output);

			counter = counter + exportBoneNodes( child , output, writing);

	return counter;


# exportar los nodos contenidos en un esqueleto.
def exportSkeletonNodes( rootJoints = None , filename = "") :

	print "-" * 30;
	print " " * 5 + "SCRIPT DE EXPORTACION DE ESQUELETOS";
	print "-" * 30;

	if rootJoints == None:
		rootJoints = cmds.ls(sl=True, type='transform');

	if rootJoints == None:
		raise RuntimeError, 'Select a root node to process';

	if filename == "":
		filename = tkFileDialog.asksaveasfilename(initialdir="D:\\phd_dev\\Data\\models\\");

	if filename == "":
		raise RuntimeError, 'No filename for store Node positions';


	output = open(filename, 'w');

	counter = 0;
	for root in rootJoints:
		counter += exportBoneNodes( root , output, False);

	output.write("%d \n" % counter); 

	counter = 0;
	for root in rootJoints:
		counter += exportBoneNodes( root , output, True);

	output.close();

	print "-" * 30;
	print " " * 5 + "FIN DE EXPORTACION";
	print "-" * 30;



# exportar los datos de un nodo.
def exportBoneData( jointNode, childsCount, output) :
	jointName = cmds.joint( jointNode, query = True, name = True );
	#pos = cmds.xform( jointNode, query = True, worldSpace = True,  translation = True);
	pos = cmds.getAttr(jointName+".translate");
	rot = cmds.getAttr(jointName+".rotate");
	ojoint = cmds.getAttr(jointName+".jointOrient");
	worldPos = cmds.xform( jointNode, query = True, worldSpace = True,  translation = True);
	worldOrient = cmds.xform( jointNode, query = True, worldSpace = True,  orientation = True);

	print pos;
	print rot;
	output.write("%s %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %d\n" % (jointName, pos[0][0], pos[0][1], pos[0][2], rot[0][0], rot[0][1], rot[0][2], ojoint[0][0], ojoint[0][1], ojoint[0][2], worldPos[0], worldPos[1], worldPos[2], worldOrient[0], worldOrient[1], worldOrient[2], childsCount));

# exportar los nodos de un hueso y sus hijos sucesivamente
def exportBone( rootJoint , output, writing) :
	children = cmds.listRelatives(rootJoint, children=True, type='joint', fullPath=True);
	counter = 1;
	childCounter = 0;

	if children:
		childCounter = len(children);

	exportBoneData(rootJoint,childCounter, output);

	if children:
		for child in children:
			counter += exportBone( child , output, writing);

	return counter;

# exportar los nodos contenidos en un esqueleto.
def exportSkeletonHierarchy( rootJoints = None , filename = "") :

	print "-" * 30;
	print " " * 5 + "SCRIPT DE EXPORTACION DE ESQUELETOS";
	print "-" * 30;

	if rootJoints == None:
		rootJoints = cmds.ls(sl=True, type='joint');

	if rootJoints == None:
		raise RuntimeError, 'Select a root node to process';

	if filename == "":
		filename = tkFileDialog.asksaveasfilename(initialdir="D:\\phd_dev\\Data\\models\\");

	if filename == "":
		raise RuntimeError, 'No filename for store Node positions';

	output = open(filename, 'w');

	output.write("%d\n" % len(rootJoints));

	counter = 0;
	for root in rootJoints:
		# print cmds.objectType( rootJoints )
		counter += exportBone( root , output, True);

	output.close();

	print "-" * 30;
	print " " * 5 + "FIN DE EXPORTACION";
	print " " * 5 + "%d huesos" % counter;
	print "-" * 30;



def exportOFFMesh( mesh = None, filename = "" ):

	print "-" * 30;
	print " " * 5 + "EXPORTACION DE MAYA EN FORMATO OFF";
	print "-" * 30;


	if mesh == None:
		mesh = cmds.ls(sl=True, type='transform');

	if mesh == None:
		raise RuntimeError, 'Select a mesh to export';

	if filename == "":
		filename = tkFileDialog.asksaveasfilename(initialdir="D:\\phd_dev\\Data\\models\\");

	if filename == "":
		raise RuntimeError, 'No filename for store Node positions';

	output = open(filename, 'w');

	OFFexport.exportMesh(output)

	output.close();

	print "-" * 30;
	print " " * 5 + "FIN EXPORTACION";
	print "-" * 30;

