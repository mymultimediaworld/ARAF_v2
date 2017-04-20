/*
*			GPAC - Multimedia Framework C SDK
*
*			Copyright (c) Jean Le Feuvre 2000-2005
*					All rights reserved
*
*  This file is part of GPAC / Scene Compositor sub-project
*
*  GPAC is free software; you can redistribute it and/or modify
*  it under the terms of the GNU Lesser General Public License as published by
*  the Free Software Foundation; either version 2, or (at your option)
*  any later version.
*   
*  GPAC is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU Lesser General Public License for more details.
*   
*  You should have received a copy of the GNU Lesser General Public
*  License along with this library; see the file COPYING.  If not, write to
*  the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA. 
*
*/

#include <gpac/modules/hardcoded_proto.h>
#include <gpac/internal/terminal_dev.h>
#include <gpac/internal/scenegraph_dev.h>
#include <gpac/internal/compositor_dev.h>

#include "loc_img_vuforia.h"

#include <QCAR/QCAR.h>
#include <QCAR/Tracker.h>
#include <QCAR/DataSet.h>
#include <QCAR/State.h>
#include <QCAR/Renderer.h>
#include <QCAR/Matrices.h>
#include <QCAR/Tool.h>
#include <QCAR/CameraDevice.h>

#include <QCAR/TrackerManager.h>
#include <QCAR/ObjectTracker.h>
#include <QCAR/TrackableResult.h>

struct LocImgVufStack
{
	GF_Node* node;
	Bool						loaded;
	LocImgVufNode				rs;
	QCAR::ObjectTracker* 		ObjectTracker;
	QCAR::DataSet* 				dataSet;
	MFString*					vuforiaImageTargetNames;

	LocImgVufStack()
	{
		node 			= NULL;
		ObjectTracker 	= NULL;
		vuforiaImageTargetNames		= (MFString*) malloc ( 1 * sizeof ( MFString ) );
		loaded 			= GF_FALSE;
		dataSet 		= 0;
		memset(&rs, 0, sizeof(rs));
	}
};

static Bool LocImgVuf_getNode(GF_Node *node, LocImgVufNode *rc)
{
	GF_FieldInfo field;
	memset(rc, 0, sizeof(LocImgVufNode));
	rc->sgprivate = node->sgprivate;
    
	/* videoSource */
	if (gf_node_get_field(node, 0, &field) != GF_OK) return GF_FALSE;
	if (field.fieldType != GF_SG_VRML_SFSTRING) return GF_FALSE;
	rc->videoSource = (SFString *) field.far_ptr;
    
	/* targetResources */
	if (gf_node_get_field(node, 1, &field) != GF_OK) return GF_FALSE;
	if (field.fieldType != GF_SG_VRML_MFSTRING) return GF_FALSE;
	rc->targetResources = (MFString *) field.far_ptr;

	/* targetResourcesTypes */
	if (gf_node_get_field(node, 2, &field) != GF_OK) return GF_FALSE;
	if (field.fieldType != GF_SG_VRML_MFSTRING) return GF_FALSE;
	rc->targetResourcesTypes = (MFString *) field.far_ptr;
    
	/* enabled */
	if (gf_node_get_field(node, 3, &field) != GF_OK) return GF_FALSE;
	if (field.fieldType != GF_SG_VRML_SFINT32) return GF_FALSE;
	rc->enabled = (SFInt32 *) field.far_ptr;

	/* maximumDelay */
	if (gf_node_get_field(node, 4, &field) != GF_OK) return GF_FALSE;
	if (field.fieldType != GF_SG_VRML_SFINT32) return GF_FALSE;
	rc->maximumDelay = (SFInt32 *) field.far_ptr;

	/* optimalDelay */
	if (gf_node_get_field(node, 5, &field) != GF_OK) return GF_FALSE;
	if (field.fieldType != GF_SG_VRML_SFINT32) return GF_FALSE;
	rc->optimalDelay = (SFInt32 *) field.far_ptr;

	/* recognitionRegion */
	if (gf_node_get_field(node, 6, &field) != GF_OK) return GF_FALSE;
	if (field.fieldType != GF_SG_VRML_MFVEC2F) return GF_FALSE;
	rc->recognitionRegion = (MFVec2f *) field.far_ptr;
    
	/* onRecognition */
	if (gf_node_get_field(node, 7, &field) != GF_OK) return GF_FALSE;
	if (field.fieldType != GF_SG_VRML_MFINT32) return GF_FALSE;
	rc->onRecognition = (MFInt32*) field.far_ptr;
    
	/* onTranslation */
	if (gf_node_get_field(node, 8, &field) != GF_OK) return GF_FALSE;
	if (field.fieldType != GF_SG_VRML_MFVEC3F) return GF_FALSE;
	rc->onTranslation = (MFVec3f*) field.far_ptr;
    
	/* onRotation */
	if (gf_node_get_field(node, 9, &field) != GF_OK) return GF_FALSE;
	if (field.fieldType != GF_SG_VRML_MFROTATION) return GF_FALSE;
	rc->onRotation = (MFRotation*) field.far_ptr;
    
	/* onError */
	if (gf_node_get_field(node, 10, &field) != GF_OK) return GF_FALSE;
	if (field.fieldType != GF_SG_VRML_SFINT32) return GF_FALSE;
	rc->onError = (SFInt32*) field.far_ptr;
	return GF_TRUE;

/* onTargetResourcesLoaded */
	if (gf_node_get_field(node, 11, &field) != GF_OK) return GF_FALSE;
	if (field.fieldType != GF_SG_VRML_SFBOOL) return GF_FALSE;
	rc->onTargetResourcesLoaded = (SFBool*) field.far_ptr;
	return GF_TRUE;
}

void LocImgVuf_sendEventsOut(GF_Node* node) {
	GF_FieldInfo onRecognition;
	GF_FieldInfo onTranslation;
	GF_FieldInfo onRotation;
	
	gf_node_get_field(node, 7, &onRecognition);
	gf_node_get_field(node, 8, &onTranslation);
	gf_node_get_field(node, 9, &onRotation);

	gf_node_event_out(node, onRecognition.fieldIndex);
	gf_node_changed(node, &onRecognition);

	gf_node_event_out(node, onTranslation.fieldIndex);
	gf_node_changed(node, &onTranslation);

	gf_node_event_out(node, onRotation.fieldIndex);
	gf_node_changed(node, &onRotation);
}

static u32 doDetect(void* data) {

	LocImgVufStack* stack = (LocImgVufStack*) data;
	GF_Node* node = stack->node;
	if ( stack->vuforiaImageTargetNames->count != stack->rs.onRecognition->count )
	{
		stack->rs.onRecognition->count 	= stack->vuforiaImageTargetNames->count;
		stack->rs.onTranslation->count 	= stack->vuforiaImageTargetNames->count;
		stack->rs.onRotation->count 	= stack->vuforiaImageTargetNames->count;
				
		stack->rs.onTranslation->vals	= (SFVec3f*)	realloc(stack->rs.onTranslation->vals,	stack->rs.onTranslation->count	*	sizeof(SFVec3f));
		stack->rs.onRotation->vals		= (SFRotation*)	realloc(stack->rs.onRotation->vals,		stack->rs.onRotation->count		*	sizeof(SFRotation));
		stack->rs.onRecognition->vals 	= (SFInt32*)	realloc(stack->rs.onRecognition->vals,	stack->rs.onRecognition->count		*	sizeof(SFInt32));
	}
	memset(stack->rs.onRecognition->vals, 	0, stack->rs.onRecognition->count * sizeof(SFInt32));
	memset(stack->rs.onTranslation->vals, 	0, stack->rs.onTranslation->count * sizeof(SFVec3f));
	memset(stack->rs.onRotation->vals, 		0, stack->rs.onRotation->count * sizeof(SFRotation));
	// Send video to engine
	QCAR::setHint(QCAR::HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 3);
	QCAR::State state = QCAR::Renderer::getInstance().begin();

	int found = state.getNumTrackableResults();
	for(int tIdx = 0; tIdx < state.getNumTrackableResults(); tIdx++)
	{
		// Get the trackable:
		const QCAR::TrackableResult* result = state.getTrackableResult(tIdx);
		const QCAR::Trackable& trackable = result->getTrackable();
		
		QCAR::Matrix34F vuf_pose_mat = result->getPose();
		QCAR::Matrix44F modelViewMatrix = QCAR::Tool::convertPose2GLMatrix(vuf_pose_mat);

		GF_Matrix mx;
		GF_Vec tr, sc, sh;
		GF_Vec tr1;
		GF_Vec4 rot;
		for ( u32 i = 0; i < 16; i++ ) {
			mx.m[i] = FLT2FIX(modelViewMatrix.data[i]);
		}

		gf_mx_decompose(&mx, &tr, &sc, &rot, &sh);
		
		u32 index = -1;

		for ( u32 i = 0; i < stack->vuforiaImageTargetNames->count; ++i ) {
			if ( !stricmp(stack->vuforiaImageTargetNames->vals[i], trackable.getName()) ) {
				index = i;
				break;
			}
		}

		if ( index == -1 ) {
			GF_LOG(GF_LOG_ERROR, GF_LOG_MEDIA, ("[VuforiaDetector] Cannot find trackable in reference names"));
		} else {
			stack->rs.onRecognition->vals[index] = 1;
			stack->rs.onTranslation->vals[index] = tr;
			stack->rs.onRotation->vals[index] = rot;
		}
		
	}

	QCAR::Renderer::getInstance().end();
	LocImgVuf_sendEventsOut(node);	

	return 0;
}

Bool loadLocImgVuf(LocImgVufStack *stack) {
	QCAR::TrackerManager& trackerManager = QCAR::TrackerManager::getInstance();

	stack->ObjectTracker = static_cast<QCAR::ObjectTracker*>(
                    trackerManager.getTracker(QCAR::ObjectTracker::getClassType()));

	if (stack->ObjectTracker == NULL)
    {
        GF_LOG(GF_LOG_ERROR, GF_LOG_MEDIA, ("[VuforiaDetector] Failed to load tracking data set because the ObjectTracker has not been initialized."));
        return GF_FALSE;
    }
    stack->ObjectTracker->start();

    QCAR::ObjectTracker* trk = static_cast<QCAR::ObjectTracker*>(stack->ObjectTracker);

	stack->dataSet = trk->createDataSet();
	if (stack->dataSet == 0)
    {
        GF_LOG(GF_LOG_ERROR, GF_LOG_MEDIA, ("Failed to create a new tracking data."));
        return GF_FALSE;
    }

	GF_LOG(GF_LOG_INFO, GF_LOG_MEDIA, ("[VuforiaDetector] Data set created"));

	if ( !stack->dataSet->load(stack->rs.targetResources->vals[0], QCAR::DataSet::STORAGE_ABSOLUTE) ) {
		GF_LOG(GF_LOG_ERROR, GF_LOG_MEDIA, ("[VuforiaDetector] Cannot load data set"));
		return GF_FALSE;
	}

	if ( !trk->activateDataSet(stack->dataSet) ) {
		GF_LOG(GF_LOG_ERROR, GF_LOG_MEDIA, ("[VuforiaDetector] Cannot activate data set"));
		return GF_FALSE;
	}

	GF_LOG(GF_LOG_INFO, GF_LOG_MEDIA, ("[VuforiaDetector] Data set loaded successfully"));

/*	const QCAR::CameraCalibration& cameraCalibration =
								QCAR::CameraDevice::getInstance().getCameraCalibration();

	QCAR::Matrix44F m = QCAR::Tool::getProjectionGL(cameraCalibration, 2.0f, 2500.0f);
	float near   = m.data[2+3*4]/(m.data[2+2*4]-1);
	float far    = m.data[2+3*4]/(m.data[2+2*4]+1);
	float bottom = near * (m.data[1+2*4]-1)/m.data[1+1*4];
	float top    = near * (m.data[1+2*4]+1)/m.data[1+1*4];
	float left   = near * (m.data[0+2*4]-1)/m.data[0+0*4];
	float right  = near * (m.data[0+2*4]+1)/m.data[0+0*4];*/

	return GF_TRUE;
}

static void TraverseLocImgVuf(GF_Node *node, void *rs, Bool is_destroy)
{
	u32 i;
	LocImgVufStack *stack = (LocImgVufStack *)gf_node_get_private(node);
	//GF_TraverseState *tr_state = (GF_TraverseState *) rs;
	
	if (is_destroy) {
		// mo unregister 
		//gf_sc_texture_destroy(&(stack->refreshTextureHandler));
		delete stack;
		gf_node_set_private(node, NULL);
		return;
	}
	if ( LocImgVuf_getNode(node, &(stack->rs)) ) {
		if ( (*(stack->rs.enabled)) == 0 ) {
			GF_LOG(GF_LOG_INFO, GF_LOG_MEDIA, ("[LocImg Vuforia] LocImg is not enabled"));
			return;
		}

		if ( !stack->loaded )
		{
			if ( loadLocImgVuf(stack) ) {
				stack->loaded = GF_TRUE;
			}
			else {
				gf_node_dirty_reset(node, GF_TRUE);
				return;
			}
		}
		doDetect(stack);

		gf_node_dirty_set(node, GF_SG_NODE_DIRTY, GF_TRUE);
	}
}

Bool compositor_init_loc_img_vuforia(GF_HardcodedProto* itfs, GF_Compositor *compositor, GF_Node *node, const char *proto_uri, MFString *targetNames)
{
	LocImgVufNode rs;
	LocImgVufStack *stack;
	GF_Proto *proto;
	u32 i;
	const char* opt;
	proto = gf_node_get_proto(node);
	if (!proto) return GF_FALSE;
	
	if (!strcmp(proto_uri, "urn:inet:gpac:builtin:LocImg"))
		if (LocImgVuf_getNode(node, &rs)) {
			stack = new LocImgVufStack();
			stack->node = node;

			stack->vuforiaImageTargetNames = targetNames;
			LocImgVuf_getNode(node, &(stack->rs));
			
			gf_node_set_private(node, stack);
			gf_node_set_callback_function(node, TraverseLocImgVuf);
			gf_node_proto_set_grouping(node);

			return GF_TRUE;
		}
	return GF_FALSE;
}

#include <jni.h>

#ifdef __cplusplus
extern "C"
{
#endif

JNIEXPORT int JNICALL
Java_com_gpac_Osmo4_vuforiaInit_initTracker(JNIEnv *, jobject)
{
	// Initialize the image tracker:
	QCAR::TrackerManager& trackerManager = QCAR::TrackerManager::getInstance();
	QCAR::Tracker* tracker = trackerManager.initTracker(QCAR::ObjectTracker::getClassType());
	if (tracker == NULL)
	{
		//LOG("Failed to initialize ObjectTracker.");
		GF_LOG(GF_LOG_ERROR, GF_LOG_MEDIA, ("[VuforiaDetector] Failed to initialize ObjectTracker."));
		return 0;
	}

	//LOG("Successfully initialized ObjectTracker.");
	GF_LOG(GF_LOG_INFO, GF_LOG_MEDIA, ("[VuforiaDetector] Successfully initialized ObjectTracker."));
	return 1;
}


JNIEXPORT void JNICALL
Java_com_gpac_Osmo4_vuforiaInit_deInitTracker(JNIEnv *, jobject)
{
	//LOG("Java_com_qualcomm_QCARSamples_ImageTargets_ImageTargets_deinitTracker");

	// Deinit the image tracker:
	QCAR::TrackerManager& trackerManager = QCAR::TrackerManager::getInstance();
	trackerManager.deinitTracker(QCAR::ObjectTracker::getClassType());
}

#ifdef __cplusplus
}
#endif
