#ifndef VUFORIA_CAMERA_H
#define VUFORIA_CAMERA_H

#include <gpac/scenegraph_vrml.h>
#include <gpac/compositor.h>
#include <gpac/modules/hardcoded_proto.h>

void compositor_init_vuforia_camera(GF_HardcodedProto* itfs, GF_Compositor *compositor, GF_Node *node);

#endif // VUFORIA_CAMERA_H