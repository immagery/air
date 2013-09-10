#Python de pruebas
import maya.cmds as cmds


def subdivideSkeleton( rootJoint , rootJointName, desiredSeperation)
	## SUBDIVISIÓN DEL ESQUELETO EN NODOS ##
	
	print "_" * 30;
	print "SCRIPT DE SUBDIVISION DE ESQUELETOS";
	print "_" * 30;

	count = 0;
	newname = rootJointName + "_Node%03d" % count;

	posX = 0;
	posY = 0;
	posZ = 0;

	cmds.joint( position = [posX, posY, posZ], name = newName);






