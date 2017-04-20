/*
*           GPAC - Multimedia Framework C SDK
*
*           Copyright (c) Jean Le Feuvre 2000-2005
*                   All rights reserved
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

//#include "vuforia_camera.h"
//#include "dx_hw.h"
#include <unistd.h>

#include "vuforia_camera.h"

#include <ctime>
#include <gpac/scenegraph_vrml.h>
#include <gpac/compositor.h>
#include <gpac/modules/hardcoded_proto.h>

#include <gpac/modules/term_ext.h>
#include <gpac/modules/hardcoded_proto.h>
#include <gpac/internal/terminal_dev.h>
#include <gpac/internal/scenegraph_dev.h>
#include "gpac/internal/compositor_dev.h"
#include <gpac/constants.h>
#include "../src/compositor/visual_manager.h"
#include "../src/compositor/visual_manager_3d.h"
#include "../src/compositor/texturing.h"
#include "../src/compositor/nodes_stacks.h"

#include <QCAR/QCAR.h>
#include <QCAR/CameraDevice.h>
#include <QCAR/Renderer.h>
#include <QCAR/VideoBackgroundConfig.h>
#include <QCAR/TrackerManager.h>
#include <QCAR/ObjectTracker.h>
#include <QCAR/Tool.h>
#include <QCAR/VideoBackgroundTextureInfo.h>
#include <QCAR/Frame.h>
#include <QCAR/State.h>
#include <QCAR/Image.h>

#include <jni.h>
#include <android/log.h>
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, "vuforia_texture", __VA_ARGS__)

#define LOG_TAG "VuforiaTexture"
#ifdef ANDROID
#  define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#  define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#else
#  define QUOTEME_(x) #x
#  define QUOTEME(x) QUOTEME_(x)
#  define LOGI(...) fprintf(stderr, "I/" LOG_TAG " (" __FILE__ ":" QUOTEME(__LINE__) "): " __VA_ARGS__)
#  define LOGE(...) fprintf(stderr, "E/" LOG_TAG "(" ")" __VA_ARGS__)
#endif




#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>

/*driver interfaces*/
#include <gpac/modules/video_out.h>
#include <gpac/list.h>
#include <gpac/constants.h>

#include <gpac/setup.h>

#ifdef GPAC_USE_GLES2
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#else
#include <GLES/gl.h>
#include <GLES/glext.h>
#endif

#define DROID_EXTREME_LOGS


/*CustomTexture: tests defining new (openGL) textures*/
typedef struct
{
    BASE_NODE
    SFBool *enabled;
} CustomTextureVuforia;

typedef struct
{
    CustomTextureVuforia tx;
    GF_TextureHandler txh;
    u32 gl_id;
    M_Shape *shape;
    const QCAR::Image *image;
    SFBool first_load;
    SFBool shapeCreated;
    SFBool textureSetup;
    SFBool wasEnabled;
    M_Layer3D *layer3d;

    float width_ratio;
    float height_ratio;
    float width_ratio_tmp;

    GLsizei imgW;
    GLsizei imgH;
} CustomTextureVuforiaStack;


void
configureVideoBackground(u32 display_width, u32 display_height)
{
    // Get the default video mode:
    QCAR::CameraDevice& cameraDevice = QCAR::CameraDevice::getInstance();
    QCAR::VideoMode videoMode = cameraDevice.
                                getVideoMode(QCAR::CameraDevice::MODE_OPTIMIZE_SPEED);


    // Configure the video background
    QCAR::VideoBackgroundConfig config;
    config.mEnabled = true;
    config.mPosition.data[0] = 0;
    config.mPosition.data[1] = 0;
    int screenHeight = display_height;
    int screenWidth = display_width;
    //hack to always go in portret mode. To be fixed!
    int isActivityInPortraitMode = 0;
    
    if (isActivityInPortraitMode)
    {
        //LOG("configureVideoBackground PORTRAIT");
        config.mSize.data[0] = videoMode.mHeight
                                * (screenHeight / (float)videoMode.mWidth);
        config.mSize.data[1] = screenHeight;

        if(config.mSize.data[0] < screenWidth)
        {
            GF_LOG(GF_LOG_ERROR, GF_LOG_MEDIA, ("Correcting rendering background size to handle missmatch between screen and video aspect ratios."));
            config.mSize.data[0] = screenWidth;
            config.mSize.data[1] = screenWidth * 
                              (videoMode.mWidth / (float)videoMode.mHeight);
        }
    }
    else
    {
        //LOG("configureVideoBackground LANDSCAPE");
        config.mSize.data[0] = screenWidth;
        config.mSize.data[1] = videoMode.mHeight
                            * (screenWidth / (float)videoMode.mWidth);

        if(config.mSize.data[1] < screenHeight)
        {
            GF_LOG(GF_LOG_ERROR, GF_LOG_MEDIA, ("Correcting rendering background size to handle missmatch between screen and video aspect ratios."));
            config.mSize.data[0] = screenHeight
                                * (videoMode.mWidth / (float)videoMode.mHeight);
            config.mSize.data[1] = screenHeight;
        }
    }

    GF_LOG(GF_LOG_ERROR, GF_LOG_MEDIA, ("Configure Video Background : Video (%d,%d), Screen (%d,%d), mSize (%d,%d)", videoMode.mWidth, videoMode.mHeight, screenWidth, screenHeight, config.mSize.data[0], config.mSize.data[1]));
    // Set the config:
    QCAR::Renderer::getInstance().setVideoBackgroundConfig(config);
}

Bool startVuforiaTexture(u32 display_width, u32 display_height) {
    // Select the camera to open, set this to QCAR::CameraDevice::CAMERA_FRONT 
    // to activate the front camera instead.
    QCAR::CameraDevice::CAMERA camera = QCAR::CameraDevice::CAMERA_BACK;

    // Initialize the camera:
    if (!QCAR::CameraDevice::getInstance().init(camera)) {
        GF_LOG(GF_LOG_ERROR, GF_LOG_MEDIA, ("[VuforiaTexture] Camera init failed"));
        return GF_FALSE;
    }


    // Select the default mode:
    if (!QCAR::CameraDevice::getInstance().selectVideoMode(
                                QCAR::CameraDevice::MODE_OPTIMIZE_SPEED)){
        GF_LOG(GF_LOG_ERROR, GF_LOG_MEDIA, ("[VuforiaTexture] Camera selectVideoMode failed"));
        return GF_FALSE;
    }

    // Configure the video background
    configureVideoBackground(display_width, display_height);

       
    // Start the camera:
    if (!QCAR::CameraDevice::getInstance().start()){
        GF_LOG(GF_LOG_ERROR, GF_LOG_MEDIA, ("[VuforiaTexture] Camera start failed"));
        return GF_FALSE;
    }

    // Uncomment to enable flash
    //if(QCAR::CameraDevice::getInstance().setFlashTorchMode(true))
    //  LOG("IMAGE TARGETS : enabled torch");

    // Uncomment to enable infinity focus mode, or any other supported focus mode
    // See CameraDevice.h for supported focus modes
    if (QCAR::CameraDevice::getInstance().setFocusMode(QCAR::CameraDevice::FOCUS_MODE_CONTINUOUSAUTO)) {
        GF_LOG(GF_LOG_INFO, GF_LOG_MEDIA, ("[VuforiaTexture] IMAGE TARGETS : enabled continuous auto focus"));
    }

    // Start the tracker:
    //QCAR::TrackerManager& trackerManager = QCAR::TrackerManager::getInstance();
    //QCAR::Tracker* imageTracker = trackerManager.getTracker(QCAR::Tracker::IMAGE_TRACKER);
    //if(imageTracker != 0)
    //    imageTracker->start();

    //GF_LOG(GF_LOG_INFO, GF_LOG_MEDIA, ("[VuforiaTexture] Start camera succesfull"));

    return GF_TRUE;
}

void stopVuforiaTexture() {
    GF_LOG(GF_LOG_INFO, GF_LOG_MEDIA, ("[VuforiaTexture] Stop a& deinit camera device"));
    QCAR::CameraDevice::getInstance().stop();
    QCAR::CameraDevice::getInstance().deinit();
}

Bool CustomTextureVuforia_GetNode(GF_Node *node, CustomTextureVuforia *tx)
{
    GF_FieldInfo field;
    memset(tx, 0, sizeof(CustomTextureVuforia));
    tx->sgprivate = node->sgprivate;
    if (gf_node_get_field(node, 0, &field) != GF_OK) return GF_FALSE;
    if (field.eventType != GF_SG_EVENT_EXPOSED_FIELD) return GF_FALSE;
    tx->enabled = (SFBool *) field.far_ptr;
    
    return GF_TRUE;
}

static void CustomTextureVuforia_update(GF_TextureHandler *txh)
{
    GF_Compositor *compositor = (GF_Compositor *)txh->compositor;

    if (txh == NULL) return;
    if (txh->owner == NULL) return;
    
    CustomTextureVuforiaStack *stack = (CustomTextureVuforiaStack *) gf_node_get_private(txh->owner);
    if (stack == NULL) return;

    if ( !(*(stack->tx.enabled))) {
        GF_LOG(GF_LOG_INFO, GF_LOG_MEDIA, ("[VuforiaTexture] Vuforia Texture is not enabled"));
        stopVuforiaTexture();
        stack->shapeCreated = GF_FALSE;
        return;
    }

    stack->width_ratio = (float)compositor->scene_width/compositor->display_width;
    stack->height_ratio = (float)compositor->scene_height/compositor->display_height;
//    GF_LOG(GF_LOG_ERROR, GF_LOG_MEDIA, ("[Vuforia] scene size (%f, %f), display size (%f, %f)", FLT2FIX(compositor->scene_width), FLT2FIX(compositor->scene_height), FLT2FIX(compositor->display_width), FLT2FIX(compositor->display_height)));
    
//    GF_SceneGraph *inScene = compositor->scene;

    if (!stack->shapeCreated) {
        stack->shapeCreated = startVuforiaTexture(compositor->display_width, compositor->display_height);
        if (!stack->shapeCreated) return;
    }

    if (!stack->textureSetup) {
        if (compositor->scene_height == 0) return;

        stack->textureSetup = GF_TRUE;

        // get the appearance node where VUFORIATEXTURE is defined
        M_Appearance *app = (M_Appearance *) gf_node_get_parent(txh->owner, 0);
        // get the shape node
        M_Shape *s = (M_Shape *) gf_node_get_parent((GF_Node *)app, 0);

        //get THIS scengraph to search if LAYER3D_VUFORIA is defined.
        // !!! compositor->scene would return the MAIN scenegraph which results in
        // not finding the node if we are in an inline node
        GF_SceneGraph *sg = gf_node_get_graph((GF_Node*)s);
        const char * shapeName = gf_node_get_name((GF_Node *)s);
        if (!(s->geometry)) {
            GF_LOG(GF_LOG_ERROR, GF_LOG_COMPOSE, ("[Vuforia] The VuforiaTexture node does not have a geometry Rectangle defined "));
            return;
        }
        if (gf_node_get_tag((GF_Node *)s)!=TAG_MPEG4_Shape) {
            GF_LOG(GF_LOG_ERROR, GF_LOG_COMPOSE, ("[Vuforia] The parent of VuforiaTexture is not a Shape node "));
            return;
        }
        if (s->geometry && (gf_node_get_tag((GF_Node *)s)==TAG_MPEG4_Shape)) {
            GF_LOG(GF_LOG_ERROR, GF_LOG_MEDIA, ("[Vuforia] Geometry node found. OK!"));
            // TODO: check if the geometry is indeed a rectangle
            if (((M_Rectangle *)s->geometry)->size.x == FLT2FIX(2)) {
                SFVec2f tmpSize;
                if ((compositor->scene_width > compositor->display_width) && (compositor->scene_height > compositor->display_height)) {
                    GF_LOG(GF_LOG_ERROR, GF_LOG_MEDIA, ("[Vuforia] Ooops, the ARAF scene size (%f, %f) is higher than the display size (%f, %f)", FLT2FIX(compositor->scene_width), FLT2FIX(compositor->scene_height), FLT2FIX(compositor->display_width), FLT2FIX(compositor->display_height)));
                    tmpSize.x = FLT2FIX(compositor->scene_width) * stack->width_ratio;
                    tmpSize.y = FLT2FIX(compositor->scene_height) * stack->width_ratio;
                }
                else {
                    tmpSize.x = FLT2FIX(compositor->scene_width);
                    tmpSize.y = FLT2FIX(compositor->scene_height);
                }
                
                GF_LOG(GF_LOG_ERROR, GF_LOG_MEDIA, ("[Vuforia] Setting the rectangle size of VuforiaTexture: (%f, %f) ", tmpSize.x, tmpSize.y));
                ((M_Rectangle *)s->geometry)->size.x = tmpSize.x;
                ((M_Rectangle *)s->geometry)->size.y = tmpSize.y;
            }
        }
        // TODO: crash if LAYER3D_VUFORIA is not found
        stack->layer3d = (M_Layer3D *)gf_sg_find_node_by_name(sg, "LAYER3D_VUFORIA");
        if (!stack->layer3d) {
            GF_LOG(GF_LOG_ERROR, GF_LOG_MEDIA, ("[Vuforia] LAYER3D_VUFORIA node was not found in the ARAF file!"));
            return;
        }
        if (stack->layer3d->size.x == FLT2FIX(-1)) {
            stack->layer3d->size.x = FLT2FIX(compositor->scene_width);
            stack->layer3d->size.y = FLT2FIX(ceil(stack->imgW * stack->width_ratio));
            GF_LOG(GF_LOG_ERROR, GF_LOG_MEDIA, ("[Vuforia] Setting the Layer3D size of VuforiaTexture: (%f, %f) ", stack->layer3d->size.x, stack->layer3d->size.y));
        }
        stack->width_ratio_tmp = stack->width_ratio;

    }

    //cover the case when the image width of the video gets updated (recompute the width of the layer3d)
    if (stack->width_ratio_tmp != stack->width_ratio) {
        GF_LOG(GF_LOG_ERROR, GF_LOG_MEDIA, ("[Vuforia] Current width ratio = %f", stack->width_ratio));
        GF_LOG(GF_LOG_ERROR, GF_LOG_MEDIA, ("[Vuforia] Previous width ratio = %f", stack->width_ratio_tmp));
        stack->width_ratio_tmp = stack->width_ratio;
        if (stack->width_ratio == stack->height_ratio) {
            GF_LOG(GF_LOG_ERROR, GF_LOG_MEDIA, ("[Vuforia] width_ratio(%f) == height_ratio(%f)", stack->width_ratio, stack->height_ratio));
            stack->layer3d->size.y = stack->imgW;
            GF_LOG(GF_LOG_ERROR, GF_LOG_MEDIA, ("[Vuforia] Setting Layer3D size y to %d", stack->imgW));
        }
        else {
            stack->layer3d->size.y = FLT2FIX(ceil(stack->imgW * stack->width_ratio));
        }
        
        stack->layer3d->size.x = FLT2FIX(compositor->scene_width);
        GF_LOG(GF_LOG_ERROR, GF_LOG_MEDIA, ("[Vuforia] RESETING the Layer3D size for VuforiaTexture: (%f, %f) ", stack->layer3d->size.x, stack->layer3d->size.y));
    }
    txh->needs_refresh = GF_TRUE;

    //alloc texture
    if (!txh->tx_io) {
        gf_sc_texture_allocate(txh);
        if (!txh->tx_io) {
            GF_LOG(GF_LOG_ERROR, GF_LOG_MEDIA, ("[Vuforia] Texture could not be allocated"));
            return;
        } 
    }
    if (!gf_sc_texture_get_gl_id(txh)) { 
        gf_sc_texture_set_data(txh);
        gf_sc_texture_push_image(txh, GF_FALSE, GF_FALSE);
        //OK we have a valid textureID
        stack->gl_id = gf_sc_texture_get_gl_id(txh);   
    }
    QCAR::Renderer::getInstance().setVideoBackgroundTextureID(stack->gl_id);
    QCAR::State state = QCAR::Renderer::getInstance().begin();
    // !!!!!!!!!!DO NOT REMOVE THE BIND COMMAND!!!!!!!!
    QCAR::Renderer::getInstance().bindVideoBackground(stack->gl_id);
    QCAR::Frame frame = state.getFrame();

    QCAR::setFrameFormat(QCAR::RGB565, true);
    u32 idx = 0;
    u32 ni = frame.getNumImages();

    for (idx; idx < ni; idx++) {
        stack->image = frame.getImage(idx);
        if (stack->image->getFormat() != QCAR::RGB565) continue;

        glBindTexture( GL_TEXTURE_2D, stack->gl_id);
        if (true) {
            stack->imgW = stack->image->getWidth();
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, stack->image->getWidth(), 
                        stack->image->getHeight(), 0, 
                        GL_RGB, GL_UNSIGNED_SHORT_5_6_5, stack->image->getPixels());
            stack->first_load = GF_FALSE;
        }
        break;
    }

// /------------------ Don't allow vuforia to display camera frames ------------------------------------------------


// ------------------ LET VUFORIA MAP THE IMAGE - NOT CORRECTLY PLACED ------------------------------------------------
/*
    QCAR::Renderer::getInstance().setVideoBackgroundTextureID(stack->gl_id);

    QCAR::State state = QCAR::Renderer::getInstance().begin();

    // !!!!!!!!!!DO NOT REMOVE THE BIND COMMAND!!!!!!!!
    QCAR::Renderer::getInstance().bindVideoBackground(stack->gl_id);
 
    glBindTexture( GL_TEXTURE_2D, stack->gl_id);
*/

// /------------------ LET VUFORIA MAP THE IMAGE - NOT CORRECTLY PLACED ------------------------------------------------
    
    QCAR::Renderer::getInstance().end();

}

static void TraverseVuforiaCamera(GF_Node *node, void *rs, Bool is_destroy)
{
    CustomTextureVuforiaStack *stack = (CustomTextureVuforiaStack *)gf_node_get_private(node);
    
    if (is_destroy) {
        GF_LOG(GF_LOG_INFO, GF_LOG_COMPOSE, ("[Vuforia] TraverseVuforiaCamera is_destroy!\n"));
        stopVuforiaTexture();
        //release texture object
        gf_sc_texture_destroy(&stack->txh);
        gf_free(stack);
        return;
    }
}

void compositor_init_vuforia_camera(GF_HardcodedProto* itfs, GF_Compositor *compositor, GF_Node *node)
{
    CustomTextureVuforia tx;
    if (CustomTextureVuforia_GetNode(node, &tx)) {
        CustomTextureVuforiaStack *stack;
        GF_SAFEALLOC(stack, CustomTextureVuforiaStack);
        if (!stack) {
            GF_LOG(GF_LOG_ERROR, GF_LOG_COMPOSE, ("[Vuforia] Failed to allocate vuforia camera stack\n"));
            return;
        }
        gf_node_set_private(node, stack);
        gf_node_set_callback_function(node, TraverseVuforiaCamera);
        stack->tx = tx;
        stack->first_load = GF_TRUE;
        stack->textureSetup = GF_FALSE;
        stack->imgW = 1280;
        stack->wasEnabled = GF_FALSE;

        //register texture object
        gf_sc_texture_setup(&stack->txh, compositor, node);
        stack->txh.update_texture_fcnt = CustomTextureVuforia_update;        
    } else {
        GF_LOG(GF_LOG_ERROR, GF_LOG_COMPOSE, ("[Vuforia] Unable to initialize custom texture\n"));
    }
}

