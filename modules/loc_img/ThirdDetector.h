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

#ifndef THIRD_DETECTOR_H
#define THIRD_DETECTOR_H

#include "IDetector.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv/cxcore.h"
#include "opencv/cv.h"

class ThirdDetectorObjectToDetect :
	public IObjectToDetect
{
public:
	ThirdDetectorObjectToDetect();
	virtual ~ThirdDetectorObjectToDetect();

	IplImage		*grayImage;
	CvSeq			*keypoints;
	CvSeq			*descriptors;
	CvMemStorage	*storage;
};

class ThirdDetector;

struct ThirdDetectorMatchingTask 
{
	double							num_matches;
	ThirdDetector*					st;
	ThirdDetectorObjectToDetect*	obj;
	CvSeq							*keypoints;
	CvSeq							*descriptors;
};

class ThirdDetector :
	public IDetector
{
public:
	ThirdDetector();
	virtual ~ThirdDetector();

	void AddImage(u32 index, char* data, u32 size, u32 width, u32 height, u32 stride, u32 color);
	s32 Detect(char* data, u32 size, u32 width, u32 height, u32 stride, u32 color, MFInt32 *onInputDetected, MFVec3f* position, MFRotation* rotation);

private:

	double *flannFindPairs( CvSeq* objectDescriptors, CvSeq* imageDescriptors, std::vector<int>& ptpairs);
	u32 DetectOneImage(void* data);
	u32 descriptor_size;
	double det_threshold;

	cv::Point* drawRectangle( std::vector<cv::KeyPoint>& keypoints_1, std::vector<cv::KeyPoint>& keypoints_2, std::vector<cv::DMatch>& filteredMatches_draw, cv::Mat img_matches, cv::Mat img_1 );
};

#endif //THIRD_DETECTOR_H
