#ifndef __MCVSURF_CPP__
#define __MCVSURF_CPP__

#include "opencv/cvaux.h"
#include "opencv/cv.h"

#include "opencv/highgui.h"
#include "vector"
#include "iostream"

CV_IMPL void McvExtractSURF( const CvArr* _img, const CvArr* _mask, 
							CvSeq** _keypoints, CvSeq** _descriptors, 
							CvMemStorage* storage, CvSURFParams params, int useProvidedKeyPts);


#endif