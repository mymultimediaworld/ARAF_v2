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

#include "SecondDetector.h"
#include <gpac/constants.h>
#include <gpac/thread.h>

SecondDetector::SecondDetector()
{
	windowSize = 25;  
	constant = 16; 
	iterations = 5;
	max_diff = 0.2f;

	structuringElement =
		cvCreateStructuringElementEx(3, 3, 1, 1, CV_SHAPE_RECT, NULL);
}

SecondDetector::~SecondDetector()
{
	cvReleaseStructuringElement(&structuringElement);
}

void SecondDetector::AddImage( u32 index, char* data, u32 size, u32 width, u32 height, u32 stride, u32 color )
{
	SecondDetectorObjectToDetect* obj;

	obj = new SecondDetectorObjectToDetect();
	AddObjectToList(obj);

	obj->index = index;

	IplImage* pImgBGR;

	if ( color == GF_PIXEL_BGR_24 )
		pImgBGR = cvCreateImageHeader(cvSize(width, height), 8, 3);
	else
		pImgBGR = cvCreateImage(cvSize(width, height), 8, 3);

	IplImage* pImgOrig;

	/*********************************/
	// TODO: Convert data to mat
	/*********************************/
	switch (color) {
		case GF_PIXEL_NV21:
			{
				pImgOrig = cvCreateImageHeader(cvSize(width, height + height / 2), 8, 1);
				cvSetData(pImgOrig, data, width);
				cvCvtColor(pImgOrig, pImgBGR, CV_YUV420sp2BGR );
				cvSetData(pImgOrig, NULL, 0);
				cvReleaseImage(&pImgOrig);
				break;
			}
		case GF_PIXEL_RGB_24:
			{
				pImgOrig = cvCreateImageHeader(cvSize(width, height), 8, 3);
				cvSetData(pImgOrig, data, stride);
				cvCvtColor(pImgOrig, pImgBGR, CV_RGB2BGR );
				cvSetData(pImgOrig, NULL, 0);
				cvReleaseImage(&pImgOrig);
				break;
			}
		case GF_PIXEL_RGBA:
			{
				pImgOrig = cvCreateImageHeader(cvSize(width, height), 8, 4);
				cvSetData(pImgOrig, data, stride);
				cvCvtColor(pImgOrig, pImgBGR, CV_RGBA2BGR );
				cvSetData(pImgOrig, NULL, 0);
				cvReleaseImage(&pImgOrig);
				break;
			}
		case GF_PIXEL_BGR_32:
			{
				pImgOrig = cvCreateImageHeader(cvSize(width, height), 8, 4);
				cvSetData(pImgOrig, data, stride);
				cvCvtColor(pImgOrig, pImgBGR, CV_BGRA2BGR );
				cvSetData(pImgOrig, NULL, 0);
				cvReleaseImage(&pImgOrig);
				break;
			}
		case GF_PIXEL_BGR_24:
			{
				cvSetData(pImgBGR, data, stride);
			}
		default:
			{
				color = color;
			}
	}

	obj->marker = cvCreateImage(cvSize(width, height), 8, 1);
	cvCvtColor(pImgBGR, obj->marker, CV_BGR2GRAY );
	if (color == GF_PIXEL_BGR_24)
		cvSetData(pImgBGR, NULL, 0);
	cvReleaseImage(&pImgBGR);

	cvAdaptiveThreshold(obj->marker, obj->marker, 255,
		CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY,
		windowSize, constant);

	if (iterations < 1) { iterations++; }
	cvNot(obj->marker, obj->marker);
	cvMorphologyEx(obj->marker, obj->marker, NULL, structuringElement, CV_MOP_CLOSE, iterations);

	cvFindContours( obj->marker, obj->storage, &(obj->contours), sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0) );
	obj->largest_contour = findLargestContour(obj->contours);
}

s32 SecondDetector::Detect( char* data, u32 size, u32 width, u32 height, u32 stride, u32 color, MFInt32 *onInputDetected, MFVec3f* position, MFRotation* rotation )
{
	
	double max_matches = 0.f;
	
	/*********************************/
	// Convert data to frame
	/*********************************/
	
	IplImage* pImgBGR;

	if ( color == GF_PIXEL_BGR_24 )
		pImgBGR = cvCreateImageHeader(cvSize(width, height), 8, 3);
	else
		pImgBGR = cvCreateImage(cvSize(width, height), 8, 3);

	IplImage* pImgOrig;
	switch (color) {
		case GF_PIXEL_NV21:
			{
				pImgOrig = cvCreateImageHeader(cvSize(width, height + height / 2), 8, 1);
				cvSetData(pImgOrig, data, width);
				cvCvtColor(pImgOrig, pImgBGR, CV_YUV420sp2BGR );
				cvSetData(pImgOrig, NULL, 0);
				cvReleaseImage(&pImgOrig);
				break;
			}
		case GF_PIXEL_YV12:
			{
				pImgOrig = cvCreateImageHeader(cvSize(width, height + height / 2), 8, 1);
				cvSetData(pImgOrig, data, width);
				cvCvtColor(pImgOrig, pImgBGR, CV_YUV420p2BGR );
				cvSetData(pImgOrig, NULL, 0);
				cvReleaseImage(&pImgOrig);
				break;
			}
		case GF_PIXEL_RGB_24:
			{
				pImgOrig = cvCreateImageHeader(cvSize(width, height), 8, 3);
				cvSetData(pImgOrig, data, stride);
				cvCvtColor(pImgOrig, pImgBGR, CV_RGB2BGR );
				cvSetData(pImgOrig, NULL, 0);
				cvReleaseImage(&pImgOrig);
				break;
			}
		case GF_PIXEL_RGBA:
			{
				pImgOrig = cvCreateImageHeader(cvSize(width, height), 8, 4);
				cvSetData(pImgOrig, data, stride);
				cvCvtColor(pImgOrig, pImgBGR, CV_RGBA2BGR );
				cvSetData(pImgOrig, NULL, 0);
				cvReleaseImage(&pImgOrig);
				break;
			}
		case GF_PIXEL_BGR_32:
			{
				pImgOrig = cvCreateImageHeader(cvSize(width, height), 8, 4);
				cvSetData(pImgOrig, data, stride);
				cvCvtColor(pImgOrig, pImgBGR, CV_BGRA2BGR );
				cvSetData(pImgOrig, NULL, 0);
				cvReleaseImage(&pImgOrig);
				break;
			}
		case GF_PIXEL_BGR_24:
			{
				cvSetData(pImgBGR, data, stride);
			}
		default:
			{
				color = color;
			}
	}

	u32 detWidth = width;///5;
	u32 detHeight = height;// - 40;

	IplImage *croppedFrame = cvCreateImage(cvSize(detWidth, detHeight), pImgBGR->depth, pImgBGR->nChannels);
	CvMemStorage* storagewrhull = cvCreateMemStorage(0);
	CvSeq *writehull = cvCreateSeq( CV_SEQ_KIND_GENERIC|CV_32SC2, sizeof(CvContour), sizeof(CvPoint), storagewrhull );
	cvSetImageROI(pImgBGR, cvRect((width/2-detWidth/2), (height/2-detHeight/2), detWidth, detHeight));
	cvCopy(pImgBGR, croppedFrame, NULL);

	if (color == GF_PIXEL_BGR_24)
		cvSetData(pImgBGR, NULL, 0);
	cvReleaseImage(&pImgBGR);

	IplImage *imgGray = 0;
	imgGray = cvCreateImage(cvSize(detWidth, detHeight), croppedFrame->depth, 1);
	cvCvtColor(croppedFrame, imgGray, CV_BGR2GRAY);
	cvReleaseImage(&croppedFrame);

	cvAdaptiveThreshold(imgGray, imgGray, 255,CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY, windowSize, constant);
	if (iterations < 1) {
		iterations++;
	}
	cvNot(imgGray, imgGray);

	cvMorphologyEx(imgGray, imgGray, NULL, structuringElement, CV_MOP_CLOSE, iterations);

	CvMemStorage* contour_storage = cvCreateMemStorage(0);
	CvSeq *contour = NULL;
	CvSeq *large_contour = NULL;
	CvSeq* hull = NULL;

	cvFindContours( imgGray, contour_storage, &contour, sizeof(CvContour), CV_RETR_LIST, /*CV_CHAIN_CODE*/CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0) );
	if(contour){
		large_contour = findLargestContour(contour); // this function may be changed something to matchShapes?!

		if(large_contour)
		{
			hull = cvConvexHull2( large_contour, 0, CV_COUNTER_CLOCKWISE, 0 );//déduit le contour convex
			CvPoint pt0;
			pt0 = **CV_GET_SEQ_ELEM( CvPoint*, hull, hull->total-1 );
			for(int i = 0; i < hull->total; i++ )
			{
				CvPoint pt = **CV_GET_SEQ_ELEM( CvPoint*, hull, i );
				pt0 = pt;
				cvSeqPush( writehull, &pt );
			}
		}
	}

	u32 listTasksSize = gf_list_count(mediaToDetectList);

	double diff = 0.0, sim = 0.0;
	double closestDistance = HUGE;
	int closestImage = -1;
	u32 i = 0;

	SecondDetectorObjectToDetect* obj = NULL;
	while ( obj = (SecondDetectorObjectToDetect*)gf_list_enum(mediaToDetectList, &i))
	{
		diff = cvMatchShapes(writehull/*large_contour*/, obj->largest_contour, CV_CONTOURS_MATCH_I3, 0 );
		if ( diff < max_diff && diff < closestDistance && diff )
		{
			closestDistance = diff;
			closestImage = i-1;
		}
	}

	if ( closestImage != -1 )
	{
		obj = (SecondDetectorObjectToDetect*)gf_list_get(mediaToDetectList, closestImage);
		closestImage = obj->index;

		onInputDetected->vals[closestImage] = 1;

		CvRect rect = cvBoundingRect(large_contour, 0);
		position->vals[closestImage].x = INT2FIX(rect.x);
		position->vals[closestImage].y = INT2FIX(-rect.y);
		position->vals[closestImage].z = 0;

		rotation->vals[closestImage].x = INT2FIX(rect.width);
		rotation->vals[closestImage].y = INT2FIX(rect.height);
		rotation->vals[closestImage].z = 0;
	}

	cvReleaseImage(&imgGray);
	cvReleaseMemStorage(&contour_storage);
	cvReleaseMemStorage(&storagewrhull);
	
	return closestImage;
}

CvSeq* SecondDetector::findLargestContour( CvSeq* contours )
{
	CvSeq* current_contour = contours;
	double largestArea = 0;
	CvSeq* largest_contour = NULL;
	double area;

	// check we at least have some contours

	if (contours == NULL){return NULL;}

	// for each contour compare it to current largest area on
	// record and remember the contour with the largest area
	// (note the use of fabs() for the cvContourArea() function)

	while (current_contour != NULL){

		area = fabs(cvContourArea(current_contour));//calcule l'aire du contour

		if(area > largestArea && area >1500){//comparer le contour aux contour actuelle ,enregisterer le contour le plus grand
			largestArea = area;
			largest_contour = current_contour;
		}

		current_contour = current_contour->h_next;
	}

	// return pointer to largest

	return largest_contour;
}

u32 SecondDetector::DetectOneImage( void* data )
{
	return 0;
}


SecondDetectorObjectToDetect::SecondDetectorObjectToDetect()
{
	marker = NULL;
	
	storage = cvCreateMemStorage(0);
	contours = NULL;
	largest_contour = NULL;
}

SecondDetectorObjectToDetect::~SecondDetectorObjectToDetect()
{
	cvReleaseMemStorage(&storage);
}
