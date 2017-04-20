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

#ifndef CAMERA_CALIBRATION_H
#define CAMERA_CALIBRATION_H

#include <gpac/scenegraph_vrml.h>
#include <gpac/compositor.h>
#include <gpac/modules/hardcoded_proto.h>

struct CameraCalibrationNode
{
	BASE_NODE

	MFString	*source;				/*exposedField*/
	
	SFBool		*enabled;				/*exposedField*/
	SFTime		*startTime;				/*exposedField*/
	SFTime		*timeBetweenSnapshots;	/*exposedField*/
	SFInt32		*snapshotCount;			/*exposedField*/
	SFVec2f		*boardSize;				/*exposedField*/
	SFInt32		*onStatus;				/*eventOut*/
};

Bool compositor_init_camera_calibration(GF_HardcodedProto* itfs, GF_Compositor *compositor, GF_Node *node);

#endif // CAMERA_CALIBRATION_H
