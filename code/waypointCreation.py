#!/usr/bin/env python

import numpy as np
import math

DENSITY = 1 #samples/meter
f = open('a_star_controls.txt', 'r')
rawNodes = f.readlines()
f.close()

nodes=[]
for i in rawNodes:
	nodes.append([float(i.split(",")[0]), float(i.split(",")[1])]) # use this for both rrt & rrt_star
	# nodes.append([float(i.split(" ")[0]), float(i.split(" ")[1])]) # use this for a_star

lastNode = nodes[0]
nodes.pop(0)
nodes.append(lastNode)

f = open('a_star_file.csv','w')

for currentNode in nodes:
	diffX = currentNode[0] - lastNode[0]
	diffY = currentNode[1] - lastNode[1]
	vertexLength = math.sqrt(diffX**2 + diffY**2)
	
	numberOfSubVertices = int(DENSITY * vertexLength)
	
	if numberOfSubVertices <=0:
		numberOfSubVertices = 1
	subVertexLengthX = diffX / numberOfSubVertices
	subVertexLengthY = diffY / numberOfSubVertices
	
	headingAngle = math.atan(diffY / diffX)
		
	for i in range(numberOfSubVertices):
		xi = lastNode[0] + subVertexLengthX*i	
		yi = lastNode[1] + subVertexLengthY*i	
		
		#representing the heading in the quatration notation
		headingZ = (1 - math.cos(headingAngle))/2
		headingW = (1 + math.cos(headingAngle))/2

		f.write(str(xi)+","+str(yi)+","+str(headingZ)+","+str(headingW)+"\n")

	lastNode = currentNode
f.close()

