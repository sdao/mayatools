//-
// ==========================================================================
// Copyright 1995,2006,2008 Autodesk, Inc. All rights reserved.
//
// Use of this software is subject to the terms of the Autodesk
// license agreement provided at the time of installation or download,
// or which otherwise accompanies this software in either electronic
// or hard copy form.
// ==========================================================================
//+

#include <maya/MTime.h>
#include <maya/MFnMesh.h>
#include <maya/MPoint.h>
#include <maya/MFloatPoint.h>
#include <maya/MFloatPointArray.h>
#include <maya/MIntArray.h>
#include <maya/MDoubleArray.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnPlugin.h>

#include <maya/MPxNode.h>
#include <maya/MPxDeformerNode.h>
#include <maya/MItGeometry.h>

#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MFnMeshData.h>
#include <maya/MFnNumericData.h>

#include <maya/MIOStream.h>

#include "helpers.h"
#include "gerstner.h"

#define MCheckErr(stat,msg)     \
if ( MS::kSuccess != stat ) {	\
cerr << msg;                    \
return MS::kFailure;            \
}

class gerstnerOcean : public MPxDeformerNode
{
public:
    gerstnerOcean() {};
	virtual 		~gerstnerOcean() {};
	static  void*	creator();
	static  MStatus initialize();
    
    // Deformation function.
    virtual MStatus deform( MDataBlock& 	block,
                            MItGeometry& 	iter,
                            const MMatrix& 	mat,
                            unsigned int 	multiIndex );
    
	static MObject	time;
    static MObject  waveSteepness;  // double
    static MObject  waveAmplitude;  // double
    static MObject  waveLength;     // double
    static MObject  waveSpeed;      // double
    static MObject  waveRotation;   // double
	static MTypeId	id;
    
protected:
	MObject createMesh(const MTime& time,
                       const double waveSteepness[3],
                       const double waveAmplitude[3],
                       const double waveLength[3],
                       const double waveSpeed[3],
                       const MVector waveDirection[3], // Vector3 with y == 0.
                       MObject& outData,
                       MStatus& stat);
};

MObject gerstnerOcean::time;
MObject gerstnerOcean::waveSteepness;
MObject gerstnerOcean::waveAmplitude;
MObject gerstnerOcean::waveLength;
MObject gerstnerOcean::waveSpeed;
MObject gerstnerOcean::waveRotation;
MTypeId gerstnerOcean::id( 0x12345 );

void* gerstnerOcean::creator()
{
	return new gerstnerOcean;
}

MStatus gerstnerOcean::initialize()
{
	MFnUnitAttribute unitAttr;
	MFnTypedAttribute typedAttr;
    MFnNumericAttribute numAttr;
    
    // Time
	gerstnerOcean::time = unitAttr.create( "time", "tm",
                                  MFnUnitAttribute::kTime,
                                          0.0 );
	addAttribute(gerstnerOcean::time);
    
    // Steepness 0 <= x <= 1.0
    gerstnerOcean::waveSteepness = numAttr.create( "waveSteepness", "st",
                                                  MFnNumericData::k3Double );
    numAttr.setDefault(.5, .4, .3);
    numAttr.setMin(0., 0., 0.);
    numAttr.setMax(1., 1., 1.);
    addAttribute(gerstnerOcean::waveSteepness);
    
    // Amplitude <= 1.0
    gerstnerOcean::waveAmplitude = numAttr.create( "waveAmplitude", "a",
                                              MFnNumericData::k3Double );
    numAttr.setDefault(.4, .2, .1);
    numAttr.setMin(0., 0., 0.);
    numAttr.setMax(10., 10., 10.);
    addAttribute(gerstnerOcean::waveAmplitude);
    
    // Length >= 2 * M_PI
    gerstnerOcean::waveLength = numAttr.create( "waveLength", "l",
                                              MFnNumericData::k3Double );
    numAttr.setDefault(400., 200., 600.);
    numAttr.setMin(10., 10., 10.);
    numAttr.setMax(1000., 1000., 1000.);
    addAttribute(gerstnerOcean::waveLength);
    
    // Speed
    gerstnerOcean::waveSpeed = numAttr.create( "waveSpeed", "sp",
                                                  MFnNumericData::k3Double );
    numAttr.setDefault(50., 100., 150.);
    numAttr.setMin(0., 0., 0.);
    numAttr.setMax(1000., 1000., 1000.);
    addAttribute(gerstnerOcean::waveSpeed);
    
    // Angle
    gerstnerOcean::waveRotation = numAttr.create( "waveAngle", "ang",
                                                  MFnNumericData::k3Double );
    numAttr.setDefault(0., 20., -20.);
    addAttribute(gerstnerOcean::waveRotation);
    
	attributeAffects(gerstnerOcean::time, gerstnerOcean::outputGeom);
    attributeAffects(gerstnerOcean::waveSteepness, gerstnerOcean::outputGeom);
    attributeAffects(gerstnerOcean::waveAmplitude, gerstnerOcean::outputGeom);
    attributeAffects(gerstnerOcean::waveLength, gerstnerOcean::outputGeom);
    attributeAffects(gerstnerOcean::waveSpeed, gerstnerOcean::outputGeom);
    attributeAffects(gerstnerOcean::waveRotation, gerstnerOcean::outputGeom);
    
	return MS::kSuccess;
}

/*!
 * Deform the point using the Gerstner wave equations.
 *
 * Arguments:
 *    block -- the datablock of the node
 *    iter -- an iterator for the geometry to be deformed
 *    m -- matrix to transform the point into world space
 *    multiIndex -- the index of the geometry that we are deforming
 */
MStatus gerstnerOcean::deform( MDataBlock& data,
                        MItGeometry& iter,
                        const MMatrix& /*m*/,
                        unsigned int /*multiIndex*/ )
{
	MStatus returnStatus = MStatus::kSuccess;
    
    // Get the time attribute.
    MDataHandle timeData = data.inputValue( time, &returnStatus );
    MCheckErr(returnStatus, "ERROR getting time data handle\n");
    MTime time = timeData.asTime();
	double seconds = time.as( MTime::kSeconds );
    
    // Get the waveSteepness attribute.
    MDataHandle steepData = data.inputValue( waveSteepness, &returnStatus );
    MCheckErr(returnStatus, "ERROR getting waveSteepness data handle\n");
    double3& steep = steepData.asDouble3();
    
    // Get the waveAmplitude attribute.
    MDataHandle ampData = data.inputValue( waveAmplitude, &returnStatus );
    MCheckErr(returnStatus, "ERROR getting waveAmplitude data handle\n");
    double3& amp = ampData.asDouble3();
    
    // Get the waveLength attribute.
    MDataHandle lengthData = data.inputValue( waveLength, &returnStatus );
    MCheckErr(returnStatus, "ERROR getting waveLength data handle\n");
    double3& length = lengthData.asDouble3();
    
    // Get the waveSpeed attribute.
    MDataHandle speedData = data.inputValue( waveSpeed, &returnStatus );
    MCheckErr(returnStatus, "ERROR getting waveSpeed data handle\n");
    double3& speed = speedData.asDouble3();
    
    // Get the waveRotation attribute.
    MDataHandle angData = data.inputValue( waveRotation, &returnStatus );
    MCheckErr(returnStatus, "ERROR getting waveRotation data handle\n");
    double3& ang = angData.asDouble3();
    
    MVector dir[3] = { MVector(cosD(ang[0]), 0., sinD(ang[0])),
        MVector(cosD(ang[1]), 0., sinD(ang[1])),
        MVector(cosD(ang[2]), 0., sinD(ang[2])) };
    
    // Determine the envelope (this is a global scale factor for the deformer).
    MDataHandle envData = data.inputValue( envelope, &returnStatus );
    MCheckErr(returnStatus, "ERROR getting envelope data handle\n");
    float env = envData.asFloat();
    
    gerstner simulation(seconds, steep, amp, length, speed, dir);
	
    // Iterate through each point in the geometry.
    for ( ; !iter.isDone(); iter.next() )
    {
        MPoint pt = iter.position();
        MPoint deformed = simulation.simulate(pt);
        
        if ( env != 1.f )
        {
            MVector diff = deformed - pt;
            deformed = pt + env * diff;
        }
        
        iter.setPosition( deformed );
    }
    
    return returnStatus;
}

MStatus initializePlugin(MObject obj)
{
	MStatus   status;
	MFnPlugin plugin(obj, PLUGIN_COMPANY, "3.0", "Any");
    
	status = plugin.registerNode("gerstnerOcean", gerstnerOcean::id,
                                 gerstnerOcean::creator, gerstnerOcean::initialize, MPxNode::kDeformerNode);
	if (!status) {
		status.perror("registerNode");
		return status;
	}
    
	return status;
}

MStatus uninitializePlugin(MObject obj)
{
	MStatus	  status;
	MFnPlugin plugin(obj);
    
	status = plugin.deregisterNode(gerstnerOcean::id);
	if (!status) {
		status.perror("deregisterNode");
		return status;
	}
    
	return status;
}
