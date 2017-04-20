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

#include "IDetector.h"

IObjectToDetect::IObjectToDetect()
{
	index = 0;
	memset(&trans, 0, sizeof(trans));
	memset(&rot, 0, sizeof(rot));
}

IObjectToDetect::~IObjectToDetect()
{
}

IDetector::IDetector()
{
	canTrack = GF_FALSE;
	isTracking = GF_FALSE;
	mediaToDetectList = gf_list_new();
	taskList = gf_list_new();
}

IDetector::~IDetector()
{
	EmptyMediaToDetectList();
	gf_list_del(taskList);
}

void IDetector::EmptyMediaToDetectList()
{
	if ( mediaToDetectList )
	{
		u32 i = 0;
		IObjectToDetect* obj;
		while ( obj = (IObjectToDetect*)gf_list_enum(mediaToDetectList, &i))
		{
			delete obj;
		}

		gf_list_del(mediaToDetectList);
		mediaToDetectList = NULL;
	}
}

void IDetector::AddObjectToList( IObjectToDetect* obj )
{
	gf_list_add(mediaToDetectList, obj);
}

void IDetector::RemoveImage( u32 index )
{
	if ( mediaToDetectList )
	{
		u32 i = 0;
		IObjectToDetect* obj;
		while ( obj = (IObjectToDetect*)gf_list_enum(mediaToDetectList, &i))
		{
			if ( obj->index == index )
			{
				gf_list_rem(mediaToDetectList, i-1);
				delete obj;
				return;
			}
		}
	}
}

void IDetector::AddTask( void* data )
{
	ThreadTask* task = new ThreadTask();
	task->det = this;
	task->th = gf_th_new(NULL);
	task->data = data;

	gf_list_add(taskList, task);
}

void IDetector::ExecuteTasks()
{
	ThreadTask* task;
	u32 i = 0;
	while ( task = (ThreadTask*)gf_list_enum(taskList, &i) )
	{	
		gf_th_run(task->th, ThreadFunc, task);
	}

	i = 0;
	while ( task = (ThreadTask*)gf_list_enum(taskList, &i) )
	{
		gf_th_stop(task->th);
		gf_th_del(task->th);
	}

	i = 0;
	while ( task = (ThreadTask*)gf_list_enum(taskList, &i) )
	{
		delete task;
	}

	gf_list_reset(taskList);
}

u32 IDetector::ThreadFunc( void* data )
{
	ThreadTask* task = (ThreadTask*)data;

	task->det->DetectOneImage(task->data);

	return 0;
}
