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

#include "ThirdDetector.h"
#include <gpac/constants.h>
#include <gpac/thread.h>

#include "Mcvsurf.h"

ThirdDetector::ThirdDetector():IDetector()
{
	descriptor_size = 128;
	det_threshold = 0.16f;
}

ThirdDetector::~ThirdDetector()
{
}

void ThirdDetector::AddImage( u32 index, char* data, u32 size, u32 width, u32 height, u32 stride, u32 color )
{
	ThirdDetectorObjectToDetect* obj;

	obj = new ThirdDetectorObjectToDetect();
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

	CvSURFParams params;

	if( descriptor_size == 64) params = cvSURFParams(1000, 0);	
	else params = cvSURFParams(1000, 1);

	McvExtractSURF(obj->grayImage, 0, &obj->keypoints, &obj->descriptors, obj->storage, params, 0);
}

double* ThirdDetector::flannFindPairs( CvSeq* objectDescriptors, CvSeq* imageDescriptors, std::vector<int>& ptpairs)
{
	int length = (int)(objectDescriptors->elem_size/sizeof(float));

	cv::Mat m_object(objectDescriptors->total, length, CV_32F);
	cv::Mat m_image(imageDescriptors->total, length, CV_32F);
	CvSeqReader obj_reader;
	float* obj_ptr = m_object.ptr<float>(0);
	cvStartReadSeq( objectDescriptors, &obj_reader,0 );

	for(int i = 0; i < objectDescriptors->total; i++ )
	{
		float* descriptor = ( float*)obj_reader.ptr;
		CV_NEXT_SEQ_ELEM( obj_reader.seq->elem_size, obj_reader );
		memcpy(obj_ptr, descriptor, length*sizeof(float));
		//if (i>objectDescriptors->total-20 && i< objectDescriptors->total) cout<<"obj_ptr : "<<*descriptor<<endl;
		//if (i < 10) cout<<"obj_ptr : "<<*descriptor<<endl;
		obj_ptr += length;
	}
	//	cout<<"\n";
	CvSeqReader img_reader;
	float* img_ptr = m_image.ptr<float>(0);
	cvStartReadSeq( imageDescriptors, &img_reader, 0 );
	for(int i = 0; i < imageDescriptors->total; i++ )
	{
		float* descriptor = ( float*)img_reader.ptr;
		CV_NEXT_SEQ_ELEM( img_reader.seq->elem_size, img_reader );
		memcpy(img_ptr, descriptor, length*sizeof(float));
		//cout<<"image_ptr : "<<*descriptor<<endl;
		img_ptr += length;
	}

	// find nearest neighbors using FLANN
	cv::Mat m_indices(objectDescriptors->total, 2, CV_32S);
	cv::Mat m_dists(objectDescriptors->total, 2, CV_32F);
	cv::flann::Index flann_index(m_image, cv::flann::KDTreeIndexParams(4));  // defaults: using 4 randomized kdtrees
	flann_index.knnSearch(m_object, m_indices, m_dists, 2, cv::flann::SearchParams(8) ); // default: 8 - maximum number of leafs checked

	int* indices_ptr = m_indices.ptr<int>(0);
	float* dists_ptr = m_dists.ptr<float>(0);
	for (int i=0;i<m_indices.rows;++i) // 0.6 by default
	{
		if (dists_ptr[2*i]<0.6*dists_ptr[2*i+1]) {
			ptpairs.push_back(i);
			ptpairs.push_back(indices_ptr[2*i]);
		}
	}

	double *sc = new double [2]; 
	sc[0] = (double)ptpairs.size()/2;
	sc[1] = sc[0]/objectDescriptors->total;

	return ( sc );
}

u32 ThirdDetector::DetectOneImage(void* data)
{
	ThirdDetectorMatchingTask* t = (ThirdDetectorMatchingTask*)data;

	std::vector<int> ptpairs;
	double* scorePairs = flannFindPairs( t->obj->descriptors, t->descriptors, ptpairs);

	t->num_matches = scorePairs[1];

	delete[] scorePairs;

	return 0;
}

s32 ThirdDetector::Detect( char* data, u32 size, u32 width, u32 height, u32 stride, u32 color, MFInt32 *onInputDetected, MFVec3f* position, MFRotation* rotation )
{
	u32 i;
	ThirdDetectorObjectToDetect* obj;

	IplImage* grayImage = cvCreateImage(cvSize(width, height), 8, 1);

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
				cvCvtColor(pImgBGR, grayImage, CV_BGR2GRAY);
				cvReleaseImage(&pImgBGR);
				cvSetData(pImgOrig, NULL, 0);
				cvReleaseImage(&pImgOrig);
				break;
			}
		case GF_PIXEL_RGB_24:
			{
				pImgOrig = cvCreateImageHeader(cvSize(width, height), 8, 3);
				cvSetData(pImgOrig, data, stride);
				cvCvtColor(pImgOrig, grayImage, CV_RGB2GRAY );
				cvSetData(pImgOrig, NULL, 0);
				cvReleaseImage(&pImgOrig);
				break;
			}
		case GF_PIXEL_RGBA:
			{
				pImgOrig = cvCreateImageHeader(cvSize(width, height), 8, 4);
				cvSetData(pImgOrig, data, stride);
				cvCvtColor(pImgOrig, grayImage, CV_RGBA2GRAY );
				cvSetData(pImgOrig, NULL, 0);
				cvReleaseImage(&pImgOrig);
				break;
			}
		case GF_PIXEL_BGR_32:
			{
				pImgOrig = cvCreateImageHeader(cvSize(width, height), 8, 4);
				cvSetData(pImgOrig, data, stride);
				cvCvtColor(pImgOrig, grayImage, CV_BGRA2GRAY );
				cvSetData(pImgOrig, NULL, 0);
				cvReleaseImage(&pImgOrig);
				break;
			}
		case GF_PIXEL_BGR_24:
			{
				pImgOrig = cvCreateImageHeader(cvSize(width, height), 8, 3);
				cvSetData(pImgOrig, data, stride);
				cvCvtColor(pImgOrig, grayImage, CV_BGR2GRAY );
				cvSetData(pImgOrig, NULL, 0);
				cvReleaseImage(&pImgOrig);
				break;
			}
		default:
			{
				color = color;
			}
	}

	int w = grayImage->width / 3;
	int h = grayImage->height / 2;
	CvRect roi;
	roi.x = grayImage->width / 2 - w / 2;
	roi.y = grayImage->height / 2 - h / 2;
	roi.width = w;
	roi.height = h;

	cvSetImageROI(grayImage, roi);
	IplImage *window = cvCreateImage(cvGetSize(grayImage), grayImage->depth, grayImage->nChannels);

	cvCopy(grayImage, window, NULL);
	cvResetImageROI(grayImage);

	cvReleaseImage(&grayImage);

	CvSURFParams params;
	CvSeq			*keypoints;
	CvSeq			*descriptors;
	CvMemStorage	*storage;

	if( descriptor_size == 64) params = cvSURFParams(1000, 0);	
	else params = cvSURFParams(1000, 1);

	storage = cvCreateMemStorage(0);

	//IplImage tmp = *imgConv;
	//cvNamedWindow("conv", CV_WINDOW_AUTOSIZE);
	//cvShowImage("conv", &tmp);
	//cvWaitKey(10);

	int64 now, then;
	double elapsed_msec, tickspermsec=cvGetTickFrequency() * 1.0e3; 

	then = cvGetTickCount();

	McvExtractSURF(window, 0, &keypoints, &descriptors, storage, params, 0);

	cvReleaseImage(&window);

	s32 matchIndex = -1;

	if (descriptors->total)
	{
		now = cvGetTickCount();
		elapsed_msec = (now-then)/tickspermsec;
		GF_LOG(GF_LOG_WARNING, GF_LOG_MEDIA, ("[ReferenceSignal] Descriptor extractor time: %.0f\n", elapsed_msec));

		u32 listTasksSize = gf_list_count(mediaToDetectList);

		ThirdDetectorMatchingTask* listTasks;
		
		listTasks = new ThirdDetectorMatchingTask[listTasksSize];
		i = 0;

		then = cvGetTickCount();

		while ( obj = (ThirdDetectorObjectToDetect*)gf_list_enum(mediaToDetectList, &i))
		{	
			listTasks[i-1].st = this;
			listTasks[i-1].obj = obj;
			listTasks[i-1].keypoints = keypoints;
			listTasks[i-1].descriptors = descriptors;
			
			AddTask(listTasks + (i - 1));
		} //end for loop over obj

		ExecuteTasks();

		std::vector<cv::DMatch> filteredMatches_draw;
		double max_matches = 0;

		for ( i = 0; i < listTasksSize; i++ )
		{
			if( listTasks[i].num_matches > max_matches ) 		
			{
				max_matches = listTasks[i].num_matches;
				matchIndex = i;
				//filteredMatches_draw = listTasks[i].filteredMatches;
			}
		}

		now = cvGetTickCount();
		elapsed_msec = (now-then)/tickspermsec;
		GF_LOG(GF_LOG_WARNING, GF_LOG_MEDIA, ("[ReferenceSignal] Matching time: %.0f, max = %f\n", elapsed_msec, max_matches));

		delete[] listTasks;

		if( max_matches > det_threshold )
		{
			obj = (ThirdDetectorObjectToDetect*)gf_list_get(mediaToDetectList, matchIndex);

			matchIndex = obj->index;

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
	}

	cvReleaseMemStorage(&storage);

	return matchIndex;
}

cv::Point* ThirdDetector::drawRectangle( std::vector<cv::KeyPoint>& keypoints_1, std::vector<cv::KeyPoint>& keypoints_2, std::vector<cv::DMatch>& filteredMatches_draw, cv::Mat img_matches, cv::Mat img_1 )
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


ThirdDetectorObjectToDetect::ThirdDetectorObjectToDetect():IObjectToDetect()
{
	grayImage		= NULL;
	keypoints	= NULL;
	descriptors = NULL;

	storage = cvCreateMemStorage(0);
}

ThirdDetectorObjectToDetect::~ThirdDetectorObjectToDetect()
{
	cvReleaseMemStorage( &storage );
	if (grayImage)
		cvReleaseImage(&grayImage);
}
