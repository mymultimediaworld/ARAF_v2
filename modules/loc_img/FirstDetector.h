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

#ifndef FIRST_DETECTOR_H
#define FIRST_DETECTOR_H

#include "IDetector.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "opencv/cxcore.h"
#include "opencv/cv.h"

class FirstDetectorObjectToDetect :
	public IObjectToDetect
{
public:
	FirstDetectorObjectToDetect();
	virtual ~FirstDetectorObjectToDetect();

	cv::Mat						*marker;
	std::vector< cv::KeyPoint >	keypoints;
	cv::Mat						descriptor;
};

class FirstDetector;

struct FirstDetectorMatchingTask 
{
	u32								num_matches;
	FirstDetector*					st;
	FirstDetectorObjectToDetect*	obj;
	std::vector<cv::KeyPoint>*		keypoints2;
	cv::Mat*						descriptors2;
	std::vector<cv::DMatch>			filteredMatches;
};

class FirstDetector :
	public IDetector
{
public:
	FirstDetector();
	virtual ~FirstDetector();

	void AddImage(u32 index, char* data, u32 size, u32 width, u32 height, u32 stride, u32 color);
	s32 Detect(char* data, u32 size, u32 width, u32 height, u32 stride, u32 color, MFInt32 *onInputDetected, MFVec3f* position, MFRotation* rotation);

	cv::Ptr<cv::FeatureDetector>		detector;
	cv::Ptr<cv::DescriptorExtractor>	descriptorExtractor;
	cv::Ptr<cv::DescriptorMatcher>		descriptorMatcher;
	cv::Ptr<cv::FeatureDetector>		detector9;
	s32									matcherFilterType;

private:
	int getMatcherFilterType( const std::string& str );
	void simpleMatching( cv::DescriptorMatcher* descriptorMatcher,
		const cv::Mat& descriptors1, const cv::Mat& descriptors2,
		std::vector<cv::DMatch>& matches12 );
	void crossCheckMatching( cv::DescriptorMatcher* descriptorMatcher,
		const cv::Mat& descriptors1, const cv::Mat& descriptors2,
		std::vector<cv::DMatch>& filteredMatches12, int knn );
	cv::Point* drawRectangle(std::vector<cv::KeyPoint>&  keypoints_1, std::vector<cv::KeyPoint>&  keypoints_2, 
		std::vector<cv::DMatch>& filteredMatches_draw, cv::Mat img_matches, cv::Mat img_1);

	//static void CalculateNumMatches(MatchingTask* t);
	//static u32 ThreadFunc(void* arg);

	u32 DetectOneImage(void* data);
};

#endif //FIRST_DETECTOR_H
