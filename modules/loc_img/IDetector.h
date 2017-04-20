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

#ifndef DETECTOR_INTERFACE_TYPE
#define DETECTOR_INTERFACE_TYPE

#include "loc_img.h"
#include <gpac/scenegraph_vrml.h>
#include <gpac/thread.h>

class IObjectToDetect
{
public:
	IObjectToDetect();
	virtual ~IObjectToDetect();

	u32			index;		// Index in the objectToDetect List
	SFVec3f		trans;		// Detected translation
	SFRotation	rot;		// Detected rotation
};

class IDetector;

struct ThreadTask 
{
	IDetector* det;
	GF_Thread* th;
	void* data;
};

class IDetector
{
public:
	IDetector();
	virtual ~IDetector();

	void EmptyMediaToDetectList();
	void RemoveImage(u32 index);

	virtual void AddImage(u32 index, char* data, u32 size, u32 width, u32 height, u32 stride, u32 color) = 0;
	virtual s32 Detect(char* data, u32 size, u32 width, u32 height, u32 stride, u32 color, MFInt32 *onInputDetected, MFVec3f* position, MFRotation* rotation) = 0;

	Bool canTrack;
	Bool isTracking;

protected:
	void AddObjectToList(IObjectToDetect* obj);

	void AddTask(void* data);
	
	void ExecuteTasks();
	static u32 ThreadFunc(void* data);
	virtual u32 DetectOneImage(void* data) = 0;

	GF_List* mediaToDetectList;
	GF_List* taskList;
};

#endif //DETECTOR_INTERFACE_TYPE
