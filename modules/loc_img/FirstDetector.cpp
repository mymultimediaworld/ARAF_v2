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

#include "FirstDetector.h"
#include <gpac/constants.h>
#include <gpac/thread.h>

enum FilterType{ NONE_FILTER = 0, CROSS_CHECK_FILTER = 1 };

FirstDetector::FirstDetector():IDetector()
{
	char* varg[5];

	varg[1] = "SURF"; 
	varg[2] = "SURF";
	varg[3] = "BruteForce";
	//varg[4] = "NoneFilter";
	varg[4] = "CrossCheckFilter";

    cv::initModule_nonfree();
	detector			= new cv::SurfFeatureDetector(400, 1, 2, false);
	detector9			= new cv::SurfFeatureDetector(400, 1, 2, false);
	descriptorExtractor	= cv::DescriptorExtractor::create( varg[2] );
	descriptorMatcher	= cv::DescriptorMatcher::create( varg[3] );

	matcherFilterType = getMatcherFilterType( varg[4] );

	if( detector.empty() || descriptorExtractor.empty() || descriptorMatcher.empty()  )
	{
		GF_LOG(GF_LOG_ERROR, GF_LOG_MEDIA, ("[ReferenceSignal] Can not create detector or descriptor exstractor or descriptor matcher of given types"));
		return;
	}
}

FirstDetector::~FirstDetector()
{
}

void FirstDetector::AddImage( u32 index, char* data, u32 size, u32 width, u32 height, u32 stride, u32 color )
{
	FirstDetectorObjectToDetect* obj;

	obj = new FirstDetectorObjectToDetect();
	AddObjectToList(obj);
	
	obj->index = index;

	if ( color == GF_PIXEL_BGR_24 )
		obj->marker = new cv::Mat(height, width, CV_8UC3, data, cv::Mat::AUTO_STEP);
	else {
		obj->marker = new cv::Mat(height, width, CV_8UC3);


		cv::Mat* pImgOrig;

		/*********************************/
		// TODO: Convert data to mat
		/*********************************/
		switch (color) {
			case GF_PIXEL_NV21:
				{
					pImgOrig = new cv::Mat(height + height / 2, width, CV_8UC1, data, cv::Mat::AUTO_STEP);
					cv::cvtColor(*pImgOrig, *(obj->marker), cv::COLOR_YUV420sp2BGR );
					delete pImgOrig;
					break;
				}
			case GF_PIXEL_RGB_24:
				{
					pImgOrig = new cv::Mat(height, width, CV_8UC3, data, cv::Mat::AUTO_STEP);
					cv::cvtColor(*pImgOrig, *(obj->marker), cv::COLOR_RGB2BGR );
					delete pImgOrig;
					break;
				}
			case GF_PIXEL_RGBA:
				{
					pImgOrig = new cv::Mat(height, width, CV_8UC4, data, cv::Mat::AUTO_STEP);
					cv::cvtColor(*pImgOrig, *(obj->marker), cv::COLOR_RGBA2BGR );
					delete pImgOrig;
					break;
				}
			case GF_PIXEL_BGR_32:
				{
					pImgOrig = new cv::Mat(height, width, CV_8UC4, data, cv::Mat::AUTO_STEP);
					cv::cvtColor(*pImgOrig, *(obj->marker), cv::COLOR_BGRA2BGR );
					delete pImgOrig;
					break;
				}
			default:
				{
					color = color;
				}
		}
	}

	detector->detect(*(obj->marker), obj->keypoints);
	descriptorExtractor->compute(*(obj->marker), obj->keypoints, obj->descriptor);
}

u32 FirstDetector::DetectOneImage(void* data)
{
	FirstDetectorMatchingTask* t = (FirstDetectorMatchingTask*)data;

	cv::Mat H12;
	double ransacReprojThreshold = 1;

	t->num_matches = 0;

	if( t->obj->keypoints.size() < 4 || t->keypoints2->size() < 4) return 0;

	switch( t->st->matcherFilterType )
	{
	case CROSS_CHECK_FILTER :
		crossCheckMatching( t->st->descriptorMatcher, t->obj->descriptor, *(t->descriptors2), t->filteredMatches, 1 );
		break;
	default :
		simpleMatching( t->st->descriptorMatcher, t->obj->descriptor, *(t->descriptors2), t->filteredMatches );
	}

	std::vector<int> queryIdxs( t->filteredMatches.size() ), trainIdxs( t->filteredMatches.size() );
	for( size_t j = 0; j < t->filteredMatches.size(); j++ )
	{
		queryIdxs[j] = t->filteredMatches[j].queryIdx;
		trainIdxs[j] = t->filteredMatches[j].trainIdx;		
	}

	double tm2 = (double)cvGetTickCount() ;
	if( ransacReprojThreshold >= 0 && t->filteredMatches.size()>3 )
	{
		std::vector<cv::Point2f> points1; cv::KeyPoint::convert(t->obj->keypoints, points1, queryIdxs);
		std::vector<cv::Point2f> points2; cv::KeyPoint::convert(*(t->keypoints2), points2, trainIdxs);
		H12 = cv::findHomography( cv::Mat(points1), cv::Mat(points2), /*CV_RANSAC*/CV_LMEDS, ransacReprojThreshold );		
	}
	tm2 = (double)cvGetTickCount() - tm2;

	if( !H12.empty() ) // filter outliers
	{
		//matchesMask.resize( filteredMatches.size() );
		std::vector<char> matchesMask(t->filteredMatches.size(),0);
		std::vector<cv::Point2f> points1; cv::KeyPoint::convert(t->obj->keypoints, points1, queryIdxs);
		std::vector<cv::Point2f> points2; cv::KeyPoint::convert(*(t->keypoints2), points2, trainIdxs);
		cv::Mat points1t; 
		perspectiveTransform(cv::Mat(points1), points1t, H12);
		double maxInlierDist = ransacReprojThreshold < 0 ? 3 : ransacReprojThreshold;
		for( size_t i1 = 0; i1 < points1.size(); i1++ )
		{
			if( norm(points2[i1] - points1t.at<cv::Point2f>((int)i1,0)) <= maxInlierDist ) // inlier
			{
				matchesMask[i1] = 1;
				t->num_matches++;
			}
		}	

		double tm3 = (double)cvGetTickCount() ;
		tm3 = (double)cvGetTickCount() - tm3;

	} // end of if( !H12.empty() ) //filter outliers

	return 0;
}

s32 FirstDetector::Detect( char* data, u32 size, u32 width, u32 height, u32 stride, u32 color, MFInt32 *onInputDetected, MFVec3f* position, MFRotation* rotation )
{
	cv::Mat *imgConv;
	u32 i;
	FirstDetectorObjectToDetect* obj;

	cv::Mat drawImg;
	
	double max_matches = 0.f;
	s32 filenameIndex = -1;

	std::vector<cv::DMatch> filteredMatches_draw;
	//std::vector<char> matchesMask_draw;
	
	CvRect objRect;

	/*********************************/
	// Convert data to frame
	/*********************************/
	cv::Mat* pImgOrig;

	if( color == GF_PIXEL_BGR_24)
	{
		imgConv = new cv::Mat(height, width, CV_8UC3, data);
	}
	else
	{
		imgConv = new cv::Mat(height, width, CV_8UC3);

		switch (color) {
		case GF_PIXEL_NV21:
			{
				pImgOrig = new cv::Mat(height + height / 2, width, CV_8UC1, data);
				cv::cvtColor(*pImgOrig, *(imgConv), cv::COLOR_YUV420sp2BGR );
				delete pImgOrig;
				break;
			}
		case GF_PIXEL_YV12:
			{
				pImgOrig = new cv::Mat(height + height / 2, width, CV_8UC1, data);
				cv::cvtColor(*pImgOrig, *(imgConv), cv::COLOR_YUV420p2BGR );
				delete pImgOrig;
				break;
			}
		case GF_PIXEL_RGB_24:
			{
				pImgOrig = new cv::Mat(height, width, CV_8UC3, data);
				cv::cvtColor(*pImgOrig, *(imgConv), cv::COLOR_RGB2BGR );
				delete pImgOrig;
				break;
			}
		case GF_PIXEL_RGBA:
			{
				pImgOrig = new cv::Mat(height, width, CV_8UC4, data);
				cv::cvtColor(*pImgOrig, *(imgConv), cv::COLOR_RGBA2BGR );
				delete pImgOrig;
				break;
			}
		case GF_PIXEL_BGR_32:
			{
				pImgOrig = new cv::Mat(height, width, CV_8UC4, data);
				cv::cvtColor(*pImgOrig, *(imgConv), cv::COLOR_BGRA2BGR );
				delete pImgOrig;
				break;
			}
		default:
			{
				color = color;
			}
		}
	}

	//IplImage tmp = *imgConv;
	//cvNamedWindow("conv", CV_WINDOW_AUTOSIZE);
	//cvShowImage("conv", &tmp);
	//cvWaitKey(10);

	int w = imgConv->cols / 3;
	int h = imgConv->rows / 2;
	cv::Rect roi;
	roi.x = imgConv->cols / 2 - w / 2;
	roi.y = imgConv->rows / 2 - h / 2;
	roi.width = w;
	roi.height = h;

	cv::Mat *window = new cv::Mat(*imgConv,roi); 

	int64 now, then;
	double elapsed_msec, tickspermsec=cvGetTickFrequency() * 1.0e3; 

	then = cvGetTickCount();

	std::vector<cv::KeyPoint> keypoints2;
	detector9->detect( *window, keypoints2 );

	cv::Mat descriptors2;
	descriptorExtractor->compute( *window, keypoints2, descriptors2 );

	now = cvGetTickCount();
	elapsed_msec = (now-then)/tickspermsec;
	GF_LOG(GF_LOG_WARNING, GF_LOG_MEDIA, ("[ReferenceSignal] Descriptor extractor time: %.0f\n", elapsed_msec));

	u32 listTasksSize = gf_list_count(mediaToDetectList);

	FirstDetectorMatchingTask* listTasks;
	
	listTasks = new FirstDetectorMatchingTask[listTasksSize];
	i = 0;

	then = cvGetTickCount();

	while ( obj = (FirstDetectorObjectToDetect*)gf_list_enum(mediaToDetectList, &i))
	{	
		listTasks[i-1].st = this;
		listTasks[i-1].obj = obj;
		listTasks[i-1].keypoints2 = &keypoints2;
		listTasks[i-1].descriptors2 = &descriptors2;
		
		AddTask(listTasks + (i - 1));
	} //end for loop over obj

	ExecuteTasks();

	for ( i = 0; i < listTasksSize; i++ )
	{
		if( listTasks[i].num_matches > 5 && listTasks[i].num_matches > max_matches ) 		
		{
			max_matches = listTasks[i].num_matches;
			filenameIndex = i;
			filteredMatches_draw = listTasks[i].filteredMatches;
		}
	}

	now = cvGetTickCount();
	elapsed_msec = (now-then)/tickspermsec;
	GF_LOG(GF_LOG_WARNING, GF_LOG_MEDIA, ("[ReferenceSignal] Matching time: %.0f\n", elapsed_msec));

	delete[] listTasks;

	if( filenameIndex > -1 )
	{
		obj = (FirstDetectorObjectToDetect*)gf_list_get(mediaToDetectList, filenameIndex);

		filenameIndex = obj->index;

		onInputDetected->vals[filenameIndex] = 1;
		/*
		drawMatches( img[filenameIndex], keypoints[filenameIndex], *imgConv, keypoints2, filteredMatches_draw, 
		drawImg, CV_RGB(0, 255, 0), CV_RGB(0, 0, 255), 
		matchesMask_draw 
		#if DRAW_RICH_KEYPOINTS_MODE, 
		DrawMatchesFlags::DRAW_RICH_KEYPOINTS
		#endif
		);*/

		cv::Point *obj_corners;
		obj_corners = drawRectangle(obj->keypoints, keypoints2, filteredMatches_draw, *imgConv, *(obj->marker));

		if ( obj_corners )
		{
			objRect.x = obj_corners[0].x; 
			objRect.y = obj_corners[0].y;
			objRect.width  = obj_corners[1].x - obj_corners[0].x;
			objRect.height = obj_corners[2].y - obj_corners[1].y;

			position->vals[filenameIndex].x = INT2FIX(objRect.x);
			position->vals[filenameIndex].y = INT2FIX(-objRect.y);
			position->vals[filenameIndex].z = 0;

			rotation->vals[filenameIndex].x = INT2FIX(objRect.width);
			rotation->vals[filenameIndex].y = INT2FIX(objRect.height);
			rotation->vals[filenameIndex].z = 0;

			delete[] obj_corners;
		}
	}

	delete imgConv;

	return filenameIndex;
}

int FirstDetector::getMatcherFilterType( const std::string& str )
{
	if( str == "NoneFilter" )
		return NONE_FILTER;
	if( str == "CrossCheckFilter" )
		return CROSS_CHECK_FILTER;
	CV_Error(CV_StsBadArg, "Invalid filter name");
	return -1;
}

void FirstDetector::simpleMatching( cv::DescriptorMatcher* descriptorMatcher, const cv::Mat& descriptors1, const cv::Mat& descriptors2, std::vector<cv::DMatch>& matches12 )
{
	std::vector<cv::DMatch> matches;
	descriptorMatcher->match( descriptors1, descriptors2, matches12 );
}

void FirstDetector::crossCheckMatching( cv::DescriptorMatcher* descriptorMatcher, const cv::Mat& descriptors1, const cv::Mat& descriptors2, std::vector<cv::DMatch>& filteredMatches12, int knn )
{
	filteredMatches12.clear();
	std::vector<std::vector<cv::DMatch> > matches12, matches21;
	descriptorMatcher->knnMatch( descriptors1, descriptors2, matches12, knn );
	descriptorMatcher->knnMatch( descriptors2, descriptors1, matches21, knn );

	for( size_t m = 0; m < matches12.size(); m++ )
	{
		bool findCrossCheck = false;
		for( size_t fk = 0; fk < matches12[m].size(); fk++ )
		{
			cv::DMatch forward = matches12[m][fk];

			for( size_t bk = 0; bk < matches21[forward.trainIdx].size(); bk++ )
			{
				cv::DMatch backward = matches21[forward.trainIdx][bk];
				if( backward.trainIdx == forward.queryIdx )
				{
					filteredMatches12.push_back(forward);
					findCrossCheck = true;
					break;
				}
			}
			if( findCrossCheck ) break;
		}
	}
}

cv::Point* FirstDetector::drawRectangle( std::vector<cv::KeyPoint>& keypoints_1, std::vector<cv::KeyPoint>& keypoints_2, std::vector<cv::DMatch>& filteredMatches_draw, cv::Mat img_matches, cv::Mat img_1 )
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





FirstDetectorObjectToDetect::FirstDetectorObjectToDetect():IObjectToDetect()
{
	marker = NULL;
}

FirstDetectorObjectToDetect::~FirstDetectorObjectToDetect()
{
	delete marker;
}
