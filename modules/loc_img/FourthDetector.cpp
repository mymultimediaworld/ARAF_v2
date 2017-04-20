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

#include "FourthDetector.h"
#include <gpac/constants.h>
#include <gpac/color.h>

#include "surf.hpp"

FourthDetector::FourthDetector():IDetector()
{
	media_minHessianVal = 2000;
	media_pointCount = 500;

	video_minHessianVal = 2000;
	video_pointCount = 100;

	key_point_threshold = 30;

	det_threshold = 0.35f;

	min_threshold_matches = 5.0f;

	draw_keypoints = GF_FALSE;

	canTrack = GF_TRUE;
	
	descriptorMatcher	= cv::DescriptorMatcher::create( "BruteForce" );
}

FourthDetector::~FourthDetector()
{
}

void FourthDetector::AddImage( u32 index, char* data, u32 size, u32 width, u32 height, u32 stride, u32 color )
{
	FourthDetectorObjectToDetect* obj;

	obj = new FourthDetectorObjectToDetect();
	AddObjectToList(obj);
	
	obj->index = index;
	
	obj->grayImage = cvCreateImage(cvSize(width, height), 8, 1);

	IplImage* pImgOrig;

	/*********************************/
	// TODO: Convert data to mat
	/*********************************/
	switch (color) {
		case GF_PIXEL_NV21:
			{
				IplImage* pImgBGR = cvCreateImage(cvSize(width, height), 8, 3);
				pImgOrig = cvCreateImageHeader(cvSize(width, height + height / 2), 8, 1);
				cvSetData(pImgOrig, data, width);
				cvCvtColor(pImgOrig, pImgBGR, CV_YUV420sp2BGR );
				cvCvtColor(pImgBGR, obj->grayImage, CV_BGR2GRAY);
				cvReleaseImage(&pImgBGR);
				cvSetData(pImgOrig, NULL, 0);
				cvReleaseImage(&pImgOrig);
				break;
			}
		case GF_PIXEL_RGB_24:
			{
				pImgOrig = cvCreateImageHeader(cvSize(width, height), 8, 3);
				cvSetData(pImgOrig, data, stride);
				cvCvtColor(pImgOrig, obj->grayImage, CV_RGB2GRAY );
				cvSetData(pImgOrig, NULL, 0);
				cvReleaseImage(&pImgOrig);
				break;
			}
		case GF_PIXEL_RGBA:
			{
				pImgOrig = cvCreateImageHeader(cvSize(width, height), 8, 4);
				cvSetData(pImgOrig, data, stride);
				cvCvtColor(pImgOrig, obj->grayImage, CV_RGBA2GRAY );
				cvSetData(pImgOrig, NULL, 0);
				cvReleaseImage(&pImgOrig);
				break;
			}
		case GF_PIXEL_BGR_32:
			{
				pImgOrig = cvCreateImageHeader(cvSize(width, height), 8, 4);
				cvSetData(pImgOrig, data, stride);
				cvCvtColor(pImgOrig, obj->grayImage, CV_BGRA2GRAY );
				cvSetData(pImgOrig, NULL, 0);
				cvReleaseImage(&pImgOrig);
				break;
			}
		case GF_PIXEL_BGR_24:
			{
				pImgOrig = cvCreateImageHeader(cvSize(width, height), 8, 3);
				cvSetData(pImgOrig, data, stride);
				cvCvtColor(pImgOrig, obj->grayImage, CV_BGR2GRAY );
				cvSetData(pImgOrig, NULL, 0);
				cvReleaseImage(&pImgOrig);
				break;
			}
		default:
			{
				color = color;
			}
	}

	int octaves = 3;
	int octaveLayers = 4;

	SURFX customSurf(media_minHessianVal, true, false, octaves, octaveLayers,
		media_pointCount);
	
	cv::Mat tmp(obj->grayImage);

	customSurf(tmp, Mat(), obj->keypoints, obj->descriptors, false);
	GF_LOG(GF_LOG_DEBUG, GF_LOG_MEDIA, ("[ReferenceSignal] Target Image INDEX: %u\n", obj->index));
	GF_LOG(GF_LOG_DEBUG, GF_LOG_MEDIA, ("[ReferenceSignal] Target Image KEYPOINT SIZE: %u\n", obj->keypoints.size()));
}

u32 FourthDetector::DetectOneImage(void* data)
{
	FourthDetectorMatchingTask* t = (FourthDetectorMatchingTask*)data;

	std::vector<cv::DMatch> matches;

	descriptorMatcher->match(t->obj->descriptors, *(t->descriptors), matches);

	t->num_matches = 0;

	for ( u32 i = 0; i < matches.size(); ++i )
		if ( matches[i].distance < det_threshold )
			t->num_matches ++;

	return 0;
}

#define RESIZE_IMAGE

s32 FourthDetector::Detect( char* data, u32 size, u32 width, u32 height, u32 stride, u32 color, MFInt32 *onInputDetected, MFVec3f* position, MFRotation* rotation )
{
	Bool gpac_conv = GF_FALSE;
	u32 i;
	FourthDetectorObjectToDetect* obj;

	IplImage* pImgOrig;

#ifndef RESIZE_IMAGE
	int window_width = width / 2;
	int window_height = height ;
#else
	int window_width = width / 3;
	int window_height = height / 3;
#endif
	
	CvRect roi;
	roi.x = width / 2 - window_width / 2;
	roi.y = height / 2 - window_height / 2;
	roi.width = window_width;
	roi.height = window_height;

	IplImage* grayImage = cvCreateImage(cvSize(window_width, window_height), 8, 1);
	IplImage* pImgBGR = NULL;

	GF_VideoSurface src;
	GF_VideoSurface dst;

	GF_Window wnd_src;
	GF_Window wnd_dst;

	/*********************************/
	// TODO: Convert data to mat
	/*********************************/

	switch (color) {
		case GF_PIXEL_RGB_24:
			{
				pImgOrig = cvCreateImageHeader(cvSize(width, height), 8, 3);
				cvSetData(pImgOrig, data, stride);

#ifndef RESIZE_IMAGE
				cvSetImageROI(pImgOrig, roi);
				cvCvtColor(pImgOrig, grayImage, CV_RGB2GRAY );
				cvResetImageROI(pImgOrig);
#else
				IplImage* resImage = cvCreateImage(cvSize(window_width, window_height), 8, 3);
				cvResize(pImgOrig, resImage);
				cvCvtColor(resImage, grayImage, CV_RGB2GRAY );
				cvReleaseImage(&resImage);
#endif
				
				if ( !draw_keypoints )
				{
					cvSetData(pImgOrig, NULL, 0);
					cvReleaseImage(&pImgOrig);
				}
				break;
			}
		case GF_PIXEL_RGBA:
			{
				pImgOrig = cvCreateImageHeader(cvSize(width, height), 8, 4);
				cvSetData(pImgOrig, data, stride);

				
#ifndef RESIZE_IMAGE
				cvSetImageROI(pImgOrig, roi);
				cvCvtColor(pImgOrig, grayImage, CV_RGBA2GRAY );
				cvResetImageROI(pImgOrig);
#else
				IplImage* resImage = cvCreateImage(cvSize(window_width, window_height), 8, 4);
				cvResize(pImgOrig, resImage);
				cvCvtColor(resImage, grayImage, CV_RGBA2GRAY );
				cvReleaseImage(&resImage);
#endif

				if ( !draw_keypoints )
				{
					cvSetData(pImgOrig, NULL, 0);
					cvReleaseImage(&pImgOrig);
				}
				break;
			}
		case GF_PIXEL_BGR_32:
			{
				pImgOrig = cvCreateImageHeader(cvSize(width, height), 8, 4);
				cvSetData(pImgOrig, data, stride);

#ifndef RESIZE_IMAGE
				cvSetImageROI(pImgOrig, roi);
				cvCvtColor(pImgOrig, grayImage, CV_BGRA2GRAY );
				cvResetImageROI(pImgOrig);
#else
				IplImage* resImage = cvCreateImage(cvSize(window_width, window_height), 8, 4);
				cvResize(pImgOrig, resImage);
				cvCvtColor(resImage, grayImage, CV_BGRA2GRAY );
				cvReleaseImage(&resImage);
#endif
				
				if ( !draw_keypoints )
				{
					cvSetData(pImgOrig, NULL, 0);
					cvReleaseImage(&pImgOrig);
				}
				break;
			}
		case GF_PIXEL_BGR_24:
			{
				pImgOrig = cvCreateImageHeader(cvSize(width, height), 8, 3);
				cvSetData(pImgOrig, data, stride);

#ifndef RESIZE_IMAGE
				cvSetImageROI(pImgOrig, roi);
				cvCvtColor(pImgOrig, grayImage, CV_BGR2GRAY );
				cvResetImageROI(pImgOrig);
#else
				IplImage* resImage = cvCreateImage(cvSize(window_width, window_height), 8, 3);
				cvResize(pImgOrig, resImage);
				cvCvtColor(resImage, grayImage, CV_BGR2GRAY );
				cvReleaseImage(&resImage);
#endif

				if ( !draw_keypoints )
				{
					cvSetData(pImgOrig, NULL, 0);
					cvReleaseImage(&pImgOrig);
				}
				break;
			}
		default:
			{
				gpac_conv = GF_TRUE;

				src.width = width;
				src.height = height;
				src.pitch_x = 0;
				src.pitch_y = stride;
				src.pixel_format = color;
				src.video_buffer = data;

				dst.width = window_width;
				dst.height = window_height;
				dst.pitch_x = 0;
				dst.pitch_y = dst.width * 3;
				dst.pixel_format = GF_PIXEL_BGR_24;
				dst.video_buffer = (char*)gf_malloc(dst.height * dst.pitch_y);

#ifndef RESIZE_IMAGE
				wnd_src.x = roi.x;
				wnd_src.y = roi.y;
				wnd_src.w = roi.width;
				wnd_src.h = roi.height;

				wnd_dst.x = 0;
				wnd_dst.y = 0;
				wnd_dst.w = roi.width;
				wnd_dst.h = roi.height;
#else
				wnd_src.x = 0;
				wnd_src.y = 0;
				wnd_src.w = width;
				wnd_src.h = height;

				wnd_dst.x = 0;
				wnd_dst.y = 0;
				wnd_dst.w = roi.width;
				wnd_dst.h = roi.height;
#endif // !RESIZE_IMAGE
				
				gf_stretch_bits(&dst, &src, &wnd_dst, &wnd_src, 255, GF_FALSE, NULL, NULL);
				
				pImgOrig = cvCreateImageHeader(cvSize(window_width, window_height), 8, 3);
				cvSetData(pImgOrig, dst.video_buffer, dst.pitch_y);

				cvCvtColor(pImgOrig, grayImage, CV_BGR2GRAY );
				
				if ( !draw_keypoints )
				{
					cvSetData(pImgOrig, NULL, 0);
					cvReleaseImage(&pImgOrig);

					gf_free(dst.video_buffer);
				}
			}
	}

	std::vector<cv::KeyPoint>	keypoints;
	cv::Mat						descriptors;
	
	//IplImage tmp = *imgConv;
	//cvNamedWindow("conv", CV_WINDOW_AUTOSIZE);
	//cvShowImage("conv", &tmp);
	//cvWaitKey(10);

	int64 now, then;
	double elapsed_msec, tickspermsec=cvGetTickFrequency() * 1.0e3; 

	then = cvGetTickCount();

	int octaves = 3;
	int octaveLayers = 4;

	SURFX customSurf(video_minHessianVal, true, false, octaves, octaveLayers,
		video_pointCount);

	cv::Mat tmp(grayImage);
	customSurf(tmp, Mat(), keypoints, descriptors, false);

	cvReleaseImage(&grayImage);

	s32 matchIndex = -1;

	if ( draw_keypoints )
	{
		if ( !gpac_conv )
		{
			for ( i = 0; i < keypoints.size(); ++i )
			{
				KeyPoint& pt = keypoints[i];
				cvCircle(pImgOrig, cvPoint((int)pt.pt.x + roi.x, (int)pt.pt.y + roi.y), 10, cvScalar(0, 0, 255, 255));
			}
			cvSetData(pImgOrig, NULL, 0);
			cvReleaseImage(&pImgOrig);
		}
		else
		{
			for ( i = 0; i < keypoints.size(); ++i )
			{
				KeyPoint& pt = keypoints[i];
				cvCircle(pImgOrig, cvPoint((int)pt.pt.x, (int)pt.pt.y), 10, cvScalar(0, 0, 255, 255));
			}
			cvSetData(pImgOrig, NULL, 0);
			cvReleaseImage(&pImgOrig);
			
			gf_stretch_bits(&src, &dst, &wnd_src, &wnd_dst, 255, GF_FALSE, NULL, NULL);
			gf_free(dst.video_buffer);
		}
	}

	if (keypoints.size() >= key_point_threshold)
	{
/*		now = cvGetTickCount();
		elapsed_msec = (now-then)/tickspermsec;
		GF_LOG(GF_LOG_DEBUG, GF_LOG_MEDIA, ("[ReferenceSignal] Descriptor extractor time: %.0f\n", elapsed_msec));*/

		u32 listTasksSize = gf_list_count(mediaToDetectList);

		FourthDetectorMatchingTask* listTasks;
		
		listTasks = new FourthDetectorMatchingTask[listTasksSize];
		i = 0;

		then = cvGetTickCount();

		while ( obj = (FourthDetectorObjectToDetect*)gf_list_enum(mediaToDetectList, &i))
		{	
			listTasks[i-1].st = this;
			listTasks[i-1].obj = obj;
			listTasks[i-1].keypoints = &keypoints;
			listTasks[i-1].descriptors = &descriptors;
			
			AddTask(listTasks + (i - 1));
		} //end for loop over obj

		ExecuteTasks();

		std::vector<cv::DMatch> filteredMatches_draw;
		u32 max_matches = 0;

		for ( i = 0; i < listTasksSize; i++ )
		{
			if( listTasks[i].num_matches > max_matches ) 		
			{
				max_matches = listTasks[i].num_matches;
				matchIndex = i;
				//filteredMatches_draw = listTasks[i].filteredMatches;
			}
		}

		if (matchIndex >= listTasksSize) {
			return -1;
		}
		if (!listTasks[matchIndex].obj->keypoints.size()) {
			return -1;
		}
		float key_size = listTasks[matchIndex].obj->keypoints.size();

		min_threshold_matches = key_size * 0.05;
		if (min_threshold_matches <= 10) {
			min_threshold_matches = 10;
		} 
		if (min_threshold_matches >= 20) {
			min_threshold_matches = 20;
		} 
		if (key_size < 100) {
			min_threshold_matches = 7;
		}
		if( float(max_matches) > min_threshold_matches)
		{
			obj = (FourthDetectorObjectToDetect*)gf_list_get(mediaToDetectList, matchIndex);

			matchIndex = obj->index;

			GF_LOG(GF_LOG_DEBUG, GF_LOG_MEDIA, ("[ReferenceSignal] Image detected! Index: %d | Matched points %f > %f  Minimum threshold  \n", matchIndex, float(max_matches), min_threshold_matches));

			onInputDetected->vals[matchIndex] = 1;

			//position->x = INT2FIX(objRect.x);
			//position->y = INT2FIX(-objRect.y);
			//position->z = 0;

			//boxsize->x = INT2FIX(objRect.width);
			//boxsize->y = INT2FIX(objRect.height);
			//boxsize->z = 0;
		}
		else 
			matchIndex = -1;

		delete[] listTasks;
	}

	return matchIndex;
}

cv::Point* FourthDetector::drawRectangle( std::vector<cv::KeyPoint>& keypoints_1, std::vector<cv::KeyPoint>& keypoints_2, std::vector<cv::DMatch>& filteredMatches_draw, cv::Mat img_matches, cv::Mat img_1 )
{
double max_dist = 0; double min_dist = 100;
	//BruteForceMatcher<L2<float>> matcher;
	//std::vector< DMatch > matches;
	//matcher.match( descriptors_1, descriptors_2, matches );
	//////-- Quick calculation of max and min distances between keypoints
	for( size_t i = 0; i < filteredMatches_draw.size(); i++ )
	{ 
		double dist = filteredMatches_draw[i].distance;
		if( dist < min_dist ) min_dist = dist;
		if( dist > max_dist ) max_dist = dist;
	}
	//
	//  //printf("-- Max dist : %f \n", max_dist );
	//  //printf("-- Min dist : %f \n", min_dist );
	//  //
	//  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
	std::vector< cv::DMatch > good_matches;

	for( size_t i = 0; i < filteredMatches_draw.size(); i++ )
	{ 
		if( filteredMatches_draw[i].distance < 3*min_dist )
		{ 
			good_matches.push_back( filteredMatches_draw[i]); 
		}
	}  

	//drawMatches( img_1, keypoints_1, img_2, keypoints_2, 
	//             good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), 
	//             vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );


	//  //-- Localize the object from img_1 in img_2 
	std::vector<cv::Point2f> obj;
	std::vector<cv::Point2f> scene;

	for( size_t i = 0; i < good_matches.size(); i++ )
	{
		//-- Get the keypoints from the good matches
		obj.push_back( keypoints_1[ good_matches[i].queryIdx ].pt );
		scene.push_back( keypoints_2[ good_matches[i].trainIdx ].pt ); 
	}

	if ( obj.size()>3 && scene.size()>3 )
	{
		cv::Mat H = findHomography( obj, scene, CV_RANSAC, 1 );
		// else {return 0};

		//-- Get the corners from the image_1 ( the object to be "detected" )
		cv::Point2f obj_corners[4] = { cvPoint(0,0), cvPoint( img_1.cols, 0 ), cvPoint( img_1.cols, img_1.rows ), cvPoint( 0, img_1.rows ) };
		cv::Point* scene_corners;
		scene_corners = new cv::Point[4];

		//-- Map these corners in the scene ( image_2)
		for( int i = 0; i < 4; i++ )
		{
			double x = obj_corners[i].x; 
			double y = obj_corners[i].y;

			double Z = 1./( H.at<double>(2,0)*x + H.at<double>(2,1)*y + H.at<double>(2,2) );
			double X = ( H.at<double>(0,0)*x + H.at<double>(0,1)*y + H.at<double>(0,2) )*Z;
			double Y = ( H.at<double>(1,0)*x + H.at<double>(1,1)*y + H.at<double>(1,2) )*Z;
			scene_corners[i] = cvPoint( cvRound(X) /*+ img_1.cols*/, cvRound(Y) );
		}  

		//-- Draw lines between the corners (the mapped object in the scene - image_2 )
		/*line( img_matches, scene_corners[0], scene_corners[1], Scalar(0, 255, 0), 2 );
		line( img_matches, scene_corners[1], scene_corners[2], Scalar( 0, 255, 0), 2 );
		line( img_matches, scene_corners[2], scene_corners[3], Scalar( 0, 255, 0), 2 );
		line( img_matches, scene_corners[3], scene_corners[0], Scalar( 0, 255, 0), 2 );*/

		return scene_corners;
	}

	return NULL;
}


FourthDetectorObjectToDetect::FourthDetectorObjectToDetect():IObjectToDetect()
{
	grayImage		= NULL;
}

FourthDetectorObjectToDetect::~FourthDetectorObjectToDetect()
{
	if (grayImage)
		cvReleaseImage(&grayImage);
}
