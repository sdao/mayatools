import sys
import math
import numpy as nmp
from scipy.misc import comb
import maya.OpenMayaMPx as OpenMayaMPx
import maya.OpenMaya as OpenMaya

PLUGIN_NODE_NAME = 'ffd1x1x1'
PLUGIN_NODE_ID = OpenMaya.MTypeId(0x9876B)

class Ffd1x1x1(OpenMayaMPx.MPxDeformerNode):
	'''A deformer that implements a 1x1x1 FFD. The eight control vertices can currently only be displaced along the y-axis.'''
	deformationTopBackAttribute = OpenMaya.MObject()
	deformationTopFrontAttribute = OpenMaya.MObject()
	deformationBottomBackAttribute = OpenMaya.MObject()
	deformationBottomFrontAttribute = OpenMaya.MObject()
	
	def __init__(self):
		OpenMayaMPx.MPxDeformerNode.__init__(self) # Must call superclass constructor.
	
	def deform(self, pDataBlock, pGeometryIterator, pLocalToWorldMatrix, pGeometryIndex):
		'''Iterates over the points of the input geometry and deforms it.'''
		envelopeValue = pDataBlock.inputValue(OpenMayaMPx.cvar.MPxDeformerNode_envelope).asFloat() # Determines "weight" of deformer on the mesh.
		inputGeometryObject = self.getDeformerInputGeometry(pDataBlock, pGeometryIndex)
		boundingBox = self.getBoundingBox(inputGeometryObject)
		
		topBack = pDataBlock.inputValue(Ffd1x1x1.deformationTopBackAttribute).asVector()
		topFront = pDataBlock.inputValue(Ffd1x1x1.deformationTopFrontAttribute).asVector()
		bottomBack = pDataBlock.inputValue(Ffd1x1x1.deformationBottomBackAttribute).asVector()
		bottomFront = pDataBlock.inputValue(Ffd1x1x1.deformationBottomFrontAttribute).asVector()
		
		bottomBackArr = [OpenMaya.MVector(0.0, 0.0 + bottomBack.x, 0.0), OpenMaya.MVector(0.0, 0.0 + bottomBack.y, 1.0)]
		topBackArr = [OpenMaya.MVector(0.0, 1.0 + topBack.x, 0.0), OpenMaya.MVector(0.0, 1.0 + topBack.y, 1.0)]
		bottomFrontArr = [OpenMaya.MVector(1.0, 0.0 + bottomFront.x, 0.0), OpenMaya.MVector(1.0, 0.0 + bottomFront.y, 1.0)]
		topFrontArr = [OpenMaya.MVector(1.0, 1.0 + topFront.x, 0.0), OpenMaya.MVector(1.0, 1.0 + topFront.y, 1.0)]
		
		displacement = nmp.array([[bottomBackArr, topBackArr], [bottomFrontArr, topFrontArr]])
		
		transform = self.getXyzToStuTransformMatrix(boundingBox)
		transformMatrix = transform.asMatrix()
		inverseMatrix = transform.asMatrixInverse()
		
		# Up = y-axis in Maya.
		while not pGeometryIterator.isDone():
			point = pGeometryIterator.position()
			
			stuPoint = point * transformMatrix # Point on lattice in STU coordinates.
			deformed = self.getDeformedPoint(stuPoint, displacement) * inverseMatrix
			
			pGeometryIterator.setPosition(deformed)
			pGeometryIterator.next()
		
	def getDeformerInputGeometry(self, pDataBlock, pGeometryIndex):
		'''Gets the mesh representation of the input geometry.'''
		inputHandle = pDataBlock.outputArrayValue(OpenMayaMPx.cvar.MPxDeformerNode_input)
		inputHandle.jumpToElement(pGeometryIndex)
		return inputHandle.outputValue().child(OpenMayaMPx.cvar.MPxDeformerNode_inputGeom).asMesh()
	
	def getBoundingBox(self, pMeshObj):
		'''Calculate a bounding box around the mesh's vertices.'''
		boundingBox = OpenMaya.MBoundingBox()
		meshFn = OpenMaya.MFnMesh(pMeshObj)
		pointArray = OpenMaya.MPointArray()
		
		# Get the points of the mesh in its local coordinate space.
		meshFn.getPoints(pointArray, OpenMaya.MSpace.kTransform) 
		
		for i in xrange(pointArray.length()):
			boundingBox.expand(pointArray[i])
			
		return boundingBox
	
	def getXyzToStuTransformMatrix(self, pBoundingBox):
		'''Returns a transformation of coordinates from object (XYZ) space into lattice (STU) space.'''
		transform = OpenMaya.MTransformationMatrix() # Transform coords into the [0...1].[0...1] range.
		
		scaleX = 1.0 / pBoundingBox.width()
		scaleY = 1.0 / pBoundingBox.height()
		scaleZ = 1.0 / pBoundingBox.depth()
		scaleUtil = OpenMaya.MScriptUtil()
		scaleUtil.createFromDouble(scaleX, scaleY, scaleZ)
		transform.addScale(scaleUtil.asDoublePtr(), OpenMaya.MSpace.kObject)
		
		boundingMinOffsetVector = OpenMaya.MPoint.origin - pBoundingBox.min()
		transform.addTranslation(boundingMinOffsetVector, OpenMaya.MSpace.kObject)
		
		return transform
	
	def getDeformedPoint(self, pStuPoint, pDisplacementLattice):
		'''Deforms an STU coordinate using the lattice, returning another STU coordinate.'''
		ffd1 = OpenMaya.MPoint()
		ffd2 = OpenMaya.MVector()
		ffd3 = OpenMaya.MVector()
		
		for i in xrange(2):
			ffd2.x = 0 # Clear out for the next summation.
			ffd2.y = 0
			ffd2.z = 0
			for j in xrange(2):
				ffd3.x = 0 # Clear out for the next summation.
				ffd3.y = 0
				ffd3.z = 0
				for k in xrange(2):
					bernstein_u = self.bernstein_poly(k, 1, pStuPoint.z) # 2 = Number of lattice points along U axis.
					lattice_point = pDisplacementLattice[i,j,k]
					ffd3 += lattice_point * bernstein_u
				bernstein_t = self.bernstein_poly(j, 1, pStuPoint.y)
				ffd2 += ffd3 * bernstein_t
			bernstein_s = self.bernstein_poly(i, 1, pStuPoint.x)
			ffd1 += ffd2 * bernstein_s
		
		return ffd1
	
	def bernstein_poly(self, i, n, t):
		'''The Bernstein polynomial of n, i as a function of t.'''
		return comb(n, i) * ((1 - t)**(n-i)) * (t**i)
	
# Maya plugin initialization functions.
def nodeCreator():
	'''Creates the deformer node and returns its pointer.'''
	return OpenMayaMPx.asMPxPtr(Ffd1x1x1())

def nodeInitializer():
	'''Initializes the attributes of the node.'''
	numericAttributeFn = OpenMaya.MFnNumericAttribute()
	
	Ffd1x1x1.deformationTopBackAttribute = numericAttributeFn.create('topBack', 'd00', OpenMaya.MFnNumericData.k2Double)
	numericAttributeFn.setMin(0.0, 0.0)
	numericAttributeFn.setMax(100.0, 100.0)
	numericAttributeFn.setStorable(True)
	numericAttributeFn.setWritable(True)
	numericAttributeFn.setReadable(False)
	Ffd1x1x1.addAttribute(Ffd1x1x1.deformationTopBackAttribute)
	
	Ffd1x1x1.deformationTopFrontAttribute = numericAttributeFn.create('topFront', 'd01', OpenMaya.MFnNumericData.k2Double)
	numericAttributeFn.setMin(0.0, 0.0)
	numericAttributeFn.setMax(100.0, 100.0)
	numericAttributeFn.setStorable(True)
	numericAttributeFn.setWritable(True)
	numericAttributeFn.setReadable(False)
	Ffd1x1x1.addAttribute(Ffd1x1x1.deformationTopFrontAttribute)
	
	Ffd1x1x1.deformationBottomBackAttribute = numericAttributeFn.create('bottomBack', 'd10', OpenMaya.MFnNumericData.k2Double)
	numericAttributeFn.setMin(0.0, 0.0)
	numericAttributeFn.setMax(100.0, 100.0)
	numericAttributeFn.setStorable(True)
	numericAttributeFn.setWritable(True)
	numericAttributeFn.setReadable(False)
	Ffd1x1x1.addAttribute(Ffd1x1x1.deformationBottomBackAttribute)
	
	Ffd1x1x1.deformationBottomFrontAttribute = numericAttributeFn.create('bottomFront', 'd11', OpenMaya.MFnNumericData.k2Double)
	numericAttributeFn.setMin(0.0, 0.0)
	numericAttributeFn.setMax(100.0, 100.0)
	numericAttributeFn.setStorable(True)
	numericAttributeFn.setWritable(True)
	numericAttributeFn.setReadable(False)
	Ffd1x1x1.addAttribute(Ffd1x1x1.deformationBottomFrontAttribute)
	
	Ffd1x1x1.attributeAffects(Ffd1x1x1.deformationTopBackAttribute, OpenMayaMPx.cvar.MPxDeformerNode_outputGeom)
	Ffd1x1x1.attributeAffects(Ffd1x1x1.deformationTopFrontAttribute, OpenMayaMPx.cvar.MPxDeformerNode_outputGeom)
	Ffd1x1x1.attributeAffects(Ffd1x1x1.deformationBottomBackAttribute, OpenMayaMPx.cvar.MPxDeformerNode_outputGeom)
	Ffd1x1x1.attributeAffects(Ffd1x1x1.deformationBottomFrontAttribute, OpenMayaMPx.cvar.MPxDeformerNode_outputGeom)
	
def initializePlugin(mObject):
	'''Registers the plugin in Maya.'''
	mPlugin = OpenMayaMPx.MFnPlugin(mObject)
	try:
		mPlugin.registerNode(PLUGIN_NODE_NAME, PLUGIN_NODE_ID, nodeCreator, nodeInitializer, OpenMayaMPx.MPxNode.kDeformerNode)
	except:
		sys.stderr.write('Could not register node: ' + PLUGIN_NODE_NAME)
		raise
		
def uninitializePlugin(mObject):
	'''Deregisters the plugin from Maya.'''
	mPlugin = OpenMayaMPx.MFnPlugin(mObject)
	try:
		mPlugin.deregisterNode(PLUGIN_NODE_ID)
	except:
		sys.stderr.write('Could not deregister node: ' + PLUGIN_NODE_NAME)
		raise