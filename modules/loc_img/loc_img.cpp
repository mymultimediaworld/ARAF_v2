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
#include <jni.h>

#include "loc_img.h"

#include "IDetector.h"
#include "FirstDetector.h"
#include "SecondDetector.h"
#include "ThirdDetector.h"
#include "FourthDetector.h"

#include "camera_calibration.h"

#include "vuforia_camera.h"
#include "loc_img_vuforia.h"

#include <android/log.h>
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, "VuforiaInit", __VA_ARGS__)

#define LOG_TAG "VuforiaInit"
#ifdef ANDROID
#  define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#  define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#else
#  define QUOTEME_(x) #x
#  define QUOTEME(x) QUOTEME_(x)
#  define LOGI(...) fprintf(stderr, "I/" LOG_TAG " (" __FILE__ ":" QUOTEME(__LINE__) "): " __VA_ARGS__)
#  define LOGE(...) fprintf(stderr, "E/" LOG_TAG "(" ")" __VA_ARGS__)
#endif

struct ImgLocStack
{
	Bool				loaded;
	Bool				resourcesLoaded;
	LocImgNode	rs;

	GF_TextureHandler	refreshTextureHandler;


	GF_MediaObject*		sourceStream;
	
	GF_MediaObject**	mediaToDetectStreams;
	u32					mediaToDetectStreamsCount;
	
	IDetector			*detector;
//	const char * 		detectionThreshold;

	u32					lastTS;

	GF_Thread*			th_detect;

	Bool				freeData;
	char*				data;
	u32					dataSize;
	u32					data_w;
	u32					data_h;
	u32					data_s;
	u32					data_pf;

	ImgLocStack()
	{
		loaded = GF_FALSE;
		resourcesLoaded = GF_FALSE;
		sourceStream = NULL;
		mediaToDetectStreams = NULL;
		mediaToDetectStreamsCount = 0;
		detector = NULL;
		lastTS = (u32)-1;

		th_detect = NULL;

		memset(&rs, 0, sizeof(rs));
	}
};

#ifdef __cplusplus
extern "C" {
#endif


static JavaVM* javaVM = 0;
static jclass sensCtrlClass;
static jobject objRef = 0;
static jobject objRef2 = 0;
static jmethodID constructorID;
static jmethodID getReferenceNames;


jint JNI_OnLoad(JavaVM* vm, void* reserved) 
{
	JNIEnv* env = 0;
  	javaVM = vm;
  	jobject objRefTmp;
//  	LOGE("[vuforiaInit] JNI_OnLoad\n");
	if ( javaVM->GetEnv((void**)&env, JNI_VERSION_1_6) != JNI_OK )
	{
		LOGE("[vuforiaInit] JNI_VERSION_1_6 != JNI_OK \n");
		return -1;
	}
		
	// Get the class and its methods in the main env
	jclass tmp = env->FindClass("com/gpac/Osmo4/vuforiaInit");
	sensCtrlClass = (jclass)env->NewGlobalRef(tmp);
	if (sensCtrlClass == 0) {
		LOGE("[vuforiaInit] Class vuforiaInit not found\n");
		return -1;
	}

	// Get Global Reference to be able to use the class
	objRefTmp = env->AllocObject(sensCtrlClass);
	objRef = env->NewGlobalRef(objRefTmp);
	if ( objRef == 0 ) {
		LOGE("[vuforiaInit] Cannot create Global Reference\n");
		return -1;
	}

	// Get the method ID for the CameraController constructor.
	constructorID = env->GetMethodID(sensCtrlClass, "<init>", "()V");
	if (constructorID == 0) {
		LOGE("[VuforiaInit] vuforiaInit Constructor not found\n");
		return -1;
	}

	getReferenceNames = env->GetMethodID(sensCtrlClass, "getReferenceNames", "(Ljava/lang/String;)Ljava/lang/String;");
	if (getReferenceNames == 0) {
		LOGE("[VuforiaInit] Function getReferenceNames not found\n");
		return -1;
	}

	return JNI_VERSION_1_6;
}

void JNI_OnUnload(JavaVM *vm, void *reserved) 
{
	JNIEnv* env = 0;
//  	LOGE("[RecorderActivity] JNI_OnUnload\n");
	if ( vm->GetEnv((void**)&env, JNI_VERSION_1_6) != JNI_OK )
		return;
	
	env->DeleteGlobalRef(objRef);
}

JavaVM* GetJavaVM() 
{
	return javaVM;
}

u32 unregisterFunc(void* data) 
{
	GetJavaVM()->DetachCurrentThread();
}

JNIEnv* GetEnvAttached() 
{
	JNIEnv* env = NULL;
	jint res = 0;
	res = GetJavaVM()->GetEnv((void**)&env, JNI_VERSION_1_6);
	if ( res == JNI_EDETACHED ) {
		LOGE("[VuforiaInit] The current thread is not attached to the VM, assuming native thread: %d\n", gf_th_id());
		GF_LOG(GF_LOG_ERROR, GF_LOG_CORE, ("[VuforiaInit] The current thread is not attached to the VM, assuming native thread: %p\n", gf_th_id()));
		if ( res = GetJavaVM()->AttachCurrentThread(&env, NULL) ) {
			GF_LOG(GF_LOG_ERROR, GF_LOG_CORE, ("[VuforiaInit] Attach current thread failed: %d\n", res));
			LOGE("[VuforiaInit] Attach current thread failed: %d\n", res);
			return NULL;
		}
		gf_register_before_exit_function(gf_th_current(), unregisterFunc);
	} else 
		if ( res == JNI_EVERSION ) {
			GF_LOG(GF_LOG_ERROR, GF_LOG_CORE, ("[VuforiaInit] The specified version is not supported\n"));
			LOGE("[VuforiaInit] The specified version is not supported\n");
			return NULL;
		}
	return env;
}

void vuforiaInit_Create() 
{
	JNIEnv* env = NULL;
	env = GetEnvAttached();
	env->PushLocalFrame(1);
	

	objRef = env->NewObject(sensCtrlClass, constructorID);
	objRef2 = env->NewGlobalRef(objRef);
	if (objRef2 == 0) 
	{
		GF_LOG(GF_LOG_ERROR, GF_LOG_CORE, ("[VuforiaInit] Cannot create VuforiaInit object\n"));
		return;
	}
}

const char* vuforiaInit_GetReferenceNames(ImgLocStack *stack) 
{
	JNIEnv* env = NULL;
	env = GetEnvAttached();
	if (stack->rs.targetResources->vals == NULL) {
		GF_LOG(GF_LOG_ERROR, GF_LOG_CORE, ("[Vuforia] The field 'targetResources' of LocImg is not defined inside the ARAF file! \n"));
	}
	jstring jStringParam = env->NewStringUTF( stack->rs.targetResources->vals[0] );

	jobject resultNames = env->CallObjectMethod(objRef, getReferenceNames, jStringParam);
	const char *strReturn = env->GetStringUTFChars((jstring)resultNames, NULL);
	return strReturn;
}


#ifdef __cplusplus
}
#endif


static Bool ImgLoc_getNode(GF_Node *node, LocImgNode *rc)
{
    GF_FieldInfo field;
	memset(rc, 0, sizeof(LocImgNode));
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

void sendEventsOut(GF_Node* node, SFBool resourcesLoaded) {
	if (resourcesLoaded) {
		GF_FieldInfo onTargetResourcesLoaded;
		gf_node_get_field(node, 11, &onTargetResourcesLoaded);
		gf_node_event_out(node, onTargetResourcesLoaded.fieldIndex);
		gf_node_changed(node, &onTargetResourcesLoaded);
		return;
	}
	
	GF_FieldInfo onRecognition;
	gf_node_get_field(node, 7, &onRecognition);
	gf_node_event_out(node, onRecognition.fieldIndex);
	gf_node_changed(node, &onRecognition);
}

static u32 doDetect(void* data) {
	ImgLocStack* stack = (ImgLocStack*) data;
	GF_Node* node = stack->refreshTextureHandler.owner;

	if ( stack->mediaToDetectStreamsCount != stack->rs.onRecognition->count )
	{
		//stack->rs.onTranslation->vals->count = stack->rs.onRotation->vals->count = 
		stack->rs.onRecognition->count = stack->mediaToDetectStreamsCount;
				
		//stack->rs.onTranslation->vals->vals		= (SFVec3f*)	realloc(stack->rs.onTranslation->vals->vals,		stack->rs.onTranslation->vals->count		*	sizeof(SFVec3f));
		//stack->rs.onRotation->vals->vals		= (SFRotation*)	realloc(stack->rs.onRotation->vals->vals,			stack->rs.onRotation->vals->count			*	sizeof(SFRotation));
		stack->rs.onRecognition->vals = (SFInt32*)	realloc(stack->rs.onRecognition->vals,	stack->rs.onRecognition->count	*	sizeof(SFInt32));
	}

	memset(stack->rs.onRecognition->vals, 0, stack->rs.onRecognition->count * sizeof(SFInt32));

	//GF_LOG(GF_LOG_WARNING, GF_LOG_MEDIA, ("[LocImg] Video frame received: %dx%d, PTS = %d\n", w, h, ts));
				
	// Send video to engine

/*	int64 now, then;
	double elapsed_msec, tickspermsec=cvGetTickFrequency() * 1.0e3; 

	then = cvGetTickCount();
*/
	MFVec3f v3f;
	v3f.vals = stack->rs.onTranslation->vals;
	MFRotation rot;
	rot.vals = stack->rs.onRotation->vals;

	s32 found = stack->detector->Detect(stack->data, stack->dataSize, stack->data_w, stack->data_h, stack->data_s, stack->data_pf, stack->rs.onRecognition, &v3f, &rot);
	sendEventsOut(node, GF_FALSE);

/*	now = cvGetTickCount();
	elapsed_msec = (now-then)/tickspermsec;
	GF_LOG(GF_LOG_INFO, GF_LOG_MEDIA, ("[LocImg] Detection time: %.0f\n", elapsed_msec));*/

	//gf_mo_release_data(stack->sourceStream, 0xFFFFFFFF, 1);

	if ( stack->freeData )
	{
		gf_free(stack->data);
		stack->data = NULL;
	}

	return 0;
}

static void UpdateTextures(GF_TextureHandler *txh)
{
	u32 i;
	u32 w, h, s, ar, pf;
	char* data;
	Bool eos;
	u32 ts, size;
	Bool images_loaded = GF_TRUE;

	GF_Node* node = txh->owner;
	ImgLocStack *stack = (ImgLocStack *)gf_node_get_private(node);

	ImgLoc_getNode(node, &(stack->rs));
    
	if ( (*(stack->rs.enabled)) == 0 ) {
		GF_LOG(GF_LOG_INFO, GF_LOG_MEDIA, ("[LocImg] LocImg is not enabled"));
		return;
	}

	if ( !stack->loaded )
	{
		MFURL murl;
		SFURL surl;
		surl.OD_ID = 0;

		murl.count = 1;
		murl.vals = &surl;
		surl.url = stack->rs.videoSource->buffer;
		stack->sourceStream = gf_scene_get_media_object_ex((GF_Scene*)gf_sg_get_private(gf_node_get_graph(node)), &murl, GF_MEDIA_OBJECT_VIDEO, GF_FALSE, NULL, GF_FALSE, node);

		if (stack->sourceStream)
		{
			stack->refreshTextureHandler.stream = stack->sourceStream;

			stack->mediaToDetectStreamsCount = stack->rs.targetResources->count;
			stack->mediaToDetectStreams = (GF_MediaObject**)gf_malloc(stack->mediaToDetectStreamsCount * sizeof(GF_MediaObject*));
			for ( i = 0; i < stack->mediaToDetectStreamsCount; ++i )
			{
				murl.count = 1;
				murl.vals = &surl;
				surl.url = stack->rs.targetResources->vals[i];
				stack->mediaToDetectStreams[i] = gf_scene_get_media_object_ex((GF_Scene*)gf_sg_get_private(gf_node_get_graph(node)), &murl, GF_MEDIA_OBJECT_VIDEO, GF_FALSE, NULL, GF_FALSE, node);
			}

			stack->loaded = GF_TRUE;
		}
	}

	if ( !stack->loaded )
		return;

	if ( stack->sourceStream->odm && stack->sourceStream->odm->state != GF_ODM_STATE_PLAY )
	{
		gf_mo_play(stack->sourceStream, 0, -1, GF_TRUE);
	}

	for ( i = 0; i < stack->mediaToDetectStreamsCount; ++i )
		if ( stack->mediaToDetectStreams[i] && stack->mediaToDetectStreams[i]->odm && stack->mediaToDetectStreams[i]->odm->state != GF_ODM_STATE_PLAY )
			gf_mo_play(stack->mediaToDetectStreams[i], 0, -1, GF_TRUE);

	for ( i = 0; i < stack->mediaToDetectStreamsCount; ++i )
	{
		if ( !stack->mediaToDetectStreams[i] || !stack->mediaToDetectStreams[i]->odm || !stack->mediaToDetectStreams[i]->odm->codec )
			continue;
		//if ( gf_mo_get_flags(stack->inputStreams[i]) & GF_MO_IS_INIT )
		{
			gf_mo_get_visual_info(stack->mediaToDetectStreams[i], &w, &h, &s, &ar, &pf, NULL);

			data = gf_mo_fetch_data(stack->mediaToDetectStreams[i], GF_MO_FETCH_RESYNC, &eos, &ts, &size, NULL, NULL, NULL);

			/*if no frame or muted don't draw*/
			if (!data || !size) {
				images_loaded = GF_FALSE;
				continue;
			}

			//TODO: Send image to engine
			stack->detector->AddImage(i, data, size, w, h, s, pf);

			GF_LOG(GF_LOG_INFO, GF_LOG_MEDIA, ("[LocImg] Image frame received: %dx%d\n", w, h ));

			gf_mo_release_data(stack->mediaToDetectStreams[i], 0xFFFFFFFF, 0);

			gf_mo_unregister(node, stack->mediaToDetectStreams[i]);
			stack->mediaToDetectStreams[i] = NULL;
		}
	}

	if ( images_loaded ) {// && gf_mo_get_flags(stack->sourceStream) & GF_MO_IS_INIT )
		if (!stack->resourcesLoaded) {
			stack->resourcesLoaded = GF_TRUE;
			stack->rs.onTargetResourcesLoaded = (SFBool*) GF_TRUE;
			sendEventsOut(node, GF_TRUE);
			return;
		}
		if ( stack->sourceStream->odm && stack->sourceStream->odm->codec )
		{
			/* Get video data*/
			gf_mo_get_visual_info(stack->sourceStream, &w, &h, &s, &ar, &pf, NULL);

			data = gf_mo_fetch_data(stack->sourceStream, GF_MO_FETCH_RESYNC, &eos, &ts, &size, NULL, NULL, NULL);

			if (!data || !size) {
				//gf_sc_invalidate(tr_state->visual->compositor, NULL);
				return;
			}

			stack->refreshTextureHandler.needs_release = GF_TRUE;

			if ( stack->lastTS == ts )
			{
				//gf_mo_release_data(stack->sourceStream, 0xFFFFFFFF, 1);
				return;
			}

			stack->lastTS = ts;

			if ( stack->th_detect ) {
				if ( gf_th_status(stack->th_detect) == GF_THREAD_STATUS_DEAD )
				{
					gf_th_del(stack->th_detect);
					stack->th_detect = NULL;
				}
				else
					return;
			}
            
			
			stack->dataSize = size;
			stack->data_w = w;
			stack->data_h = h;
			stack->data_s = s;
			stack->data_pf = pf;
            
			if ( stack->detector->canTrack && !stack->detector->isTracking  ) {
				
				stack->freeData = GF_TRUE;
				stack->data = (char*)gf_malloc(size);
				memcpy(stack->data, data, size);
				stack->th_detect = gf_th_new(NULL);
				gf_th_run(stack->th_detect, doDetect, stack);
			}
			else {
				stack->freeData = GF_FALSE;
				stack->data = data;
				doDetect(stack);
				//sendEventsOut(node);
			}
		}
	}
}

static void TraverseLocImg(GF_Node *node, void *rs, Bool is_destroy)
{
	u32 i;
	ImgLocStack *stack = (ImgLocStack *)gf_node_get_private(node);
	//GF_TraverseState *tr_state = (GF_TraverseState *) rs;
	
	if (is_destroy) {

		// mo unregister 
		gf_sc_texture_destroy(&(stack->refreshTextureHandler));

		delete stack->detector;
		stack->detector = NULL;

		for ( i = 0; i < stack->mediaToDetectStreamsCount; ++i )
		{
			if ( ! stack->mediaToDetectStreams[i] )
				continue;
			gf_mo_unregister(node, stack->mediaToDetectStreams[i]);
			stack->mediaToDetectStreams[i] = NULL;
		}

		gf_mo_unregister(node, stack->sourceStream);
		stack->sourceStream = NULL;

		gf_free(stack->mediaToDetectStreams);
		delete stack;

		gf_node_set_private(node, NULL);
		return;
	}

	//if (tr_state->traversing_mode==TRAVERSE_SORT) {
	//	if (gf_node_dirty_get(node)) {
	//gf_node_dirty_clear(node, GF_SG_NODE_DIRTY);
	//	}
	//}

	

}

Bool compositor_init_loc_img(GF_HardcodedProto* itfs, GF_Compositor *compositor, GF_Node *node, const char *proto_uri)
{
	LocImgNode rs;
	ImgLocStack *stack;
	GF_Proto *proto;
	u32 i;
	const char* opt;

	proto = gf_node_get_proto(node);
	if (!proto) return GF_FALSE;

	if (!strcmp(proto_uri, "urn:inet:gpac:builtin:VuforiaTexture")) {
		vuforiaInit_Create();
		compositor_init_vuforia_camera(itfs, compositor, node);
	}
	
	if (!strcmp(proto_uri, "urn:inet:gpac:builtin:LocImg")) {
		if (ImgLoc_getNode(node, &rs)) {

			// ------------- targetResourcesTypes = 99 => Vuforia should be invoked! -------------

			if (rs.targetResourcesTypes->vals != NULL && !strcmp(rs.targetResourcesTypes->vals[0], "99")) {
				stack = new ImgLocStack();
				ImgLoc_getNode(node, &(stack->rs));
				// get the Vuforia ImageTarget names from the XML file [Android getReferenceNames()]
				const char* targetNamesString = vuforiaInit_GetReferenceNames(stack);

				// split the result [e.g. target1 target2 target3] by space
				char * stringTmp;
				MFString* vuforiaImageTargetNames;
				u32 j = 0;

				stringTmp = strtok ((char *)targetNamesString," ");
				
				vuforiaImageTargetNames = (MFString*) malloc ( 1 * sizeof ( MFString ) );

				if ( vuforiaImageTargetNames == NULL ) {
					GF_LOG(GF_LOG_ERROR, GF_LOG_CORE, ("[LocImg Vuforia] Could not allocate memory for Vuforia target names -1- !" ) );
				}
				vuforiaImageTargetNames->vals = (char**)gf_malloc(100*sizeof(char*));
				if ( vuforiaImageTargetNames->vals == NULL ) {
					GF_LOG(GF_LOG_ERROR, GF_LOG_CORE, ("[LocImg Vuforia] Could not allocate memory for Vuforia target names -2- !" ) );
				}

				while (stringTmp != NULL) {
				  	vuforiaImageTargetNames->vals[j] = (char*)gf_malloc((strlen(stringTmp)+1)*sizeof(char));
				    strcpy(vuforiaImageTargetNames->vals[j], stringTmp);
				    GF_LOG(GF_LOG_INFO, GF_LOG_CORE, ("[LocImg Vuforia] ImageTarget name[%d] %s \n", j, vuforiaImageTargetNames->vals[j]));
				    stringTmp = strtok (NULL, " ");
				    j++;
				  }
				vuforiaImageTargetNames->count = j;

				compositor_init_loc_img_vuforia(itfs, compositor, node, proto_uri, vuforiaImageTargetNames);
			}
			// ------------- otherwise the default image target recognition is used -------------
			else {
				if ( !rs.videoSource->buffer ) {
					GF_LOG(GF_LOG_ERROR, GF_LOG_CORE, ("[LocImg ARAF] the field 'videoSource' is not defined!" ) );
					return GF_FALSE;
				}

				stack = new ImgLocStack();
//				stack->detectionThreshold = gf_modules_get_option((GF_BaseInterface*) itfs, "LocImg", "DetectionThreshold");

				ImgLoc_getNode(node, &(stack->rs));

				stack->detector = new FourthDetector();

/*				if (stack->detectionThreshold) {
					GF_LOG(GF_LOG_ERROR, GF_LOG_CORE, ("[LocImg ARAF] The detection threshold has been set [using the config file] to %s", stack->detectionThreshold ) );
					stack->detector->set_det_threshhold(atoi(stack->detectionThreshold));
				}
				else {
					stack->detector->set_det_threshhold(15);
					GF_LOG(GF_LOG_ERROR, GF_LOG_CORE, ("[LocImg ARAF] Setting the detection threshold to 15") );
				}
*/

				gf_node_set_private(node, stack);
				gf_node_set_callback_function(node, TraverseLocImg);
				gf_node_proto_set_grouping(node);

				gf_sc_texture_setup(&(stack->refreshTextureHandler), compositor, node);
				stack->refreshTextureHandler.update_texture_fcnt = UpdateTextures;
				return GF_TRUE;
			}
		}
	}

	if (!strcmp(proto_uri, "urn:inet:gpac:builtin:CameraCalibration")) {
		compositor_init_camera_calibration(itfs, compositor, node);
	}
	
	return GF_FALSE;
}

Bool can_load_proto(const char* url)
{
	if (!strcmp(url, "urn:inet:gpac:builtin:LocImg"))
		return GF_TRUE;
	if (!strcmp(url, "urn:inet:gpac:builtin:CameraCalibration"))
		return GF_TRUE;
	if (!strcmp(url, "urn:inet:gpac:builtin:VuforiaTexture")) {
		GF_LOG(GF_LOG_INFO, GF_LOG_MEDIA, ("[LocImg] can load VuforiaTexture"));
		return GF_TRUE;
	}
	return GF_FALSE;
}

GF_HardcodedProto* NewLocImg()
{
	GF_HardcodedProto *ifcd;
	GF_SAFEALLOC(ifcd, GF_HardcodedProto);

	GF_REGISTER_MODULE_INTERFACE(ifcd, GF_HARDCODED_PROTO_INTERFACE, "LocImg Proto", "gpac distribution");

	ifcd->init = compositor_init_loc_img;
	ifcd->can_load_proto = can_load_proto;

	return ifcd;
}

void DeleteLocImg(GF_HardcodedProto* itf)
{
	gf_free(itf);
}

#ifdef __cplusplus
extern "C" {
#endif
    
GPAC_MODULE_EXPORT
	const u32 *QueryInterfaces() 
	{
		static u32 si [] = {
			GF_HARDCODED_PROTO_INTERFACE,
			0
		};
		return si; 
	}
GPAC_MODULE_EXPORT
	GF_BaseInterface *LoadInterface(u32 InterfaceType) 
	{
		if (InterfaceType == GF_HARDCODED_PROTO_INTERFACE) return (GF_BaseInterface *)NewLocImg();
		return NULL;
	}
GPAC_MODULE_EXPORT
	void ShutdownInterface(GF_BaseInterface *ifce)
	{
		switch (ifce->InterfaceType) {
	case GF_HARDCODED_PROTO_INTERFACE: 
		DeleteLocImg((GF_HardcodedProto*)ifce);
		break;
		}
	}
    
GPAC_MODULE_STATIC_DECLARATION( loc_img )

#ifdef __cplusplus
}
#endif
