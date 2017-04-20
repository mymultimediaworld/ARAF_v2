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

#include "camera_calibration.h"

#include <gpac/modules/hardcoded_proto.h>
#include <gpac/internal/terminal_dev.h>
#include <gpac/internal/scenegraph_dev.h>
#include <gpac/internal/compositor_dev.h>

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

struct CameraCalibrationStack
{
	Bool					loaded;
	CameraCalibrationNode	cc;

	GF_TextureHandler		refreshTextureHandler;
	GF_MediaObject*			sourceStream;

	u32						lastTS;
	u32						lastShot;

	std::vector<std::vector<cv::Point2f> > imagePoints;

	GF_HardcodedProto*		itfs;

	CameraCalibrationStack()
	{
		loaded = GF_FALSE;
		sourceStream = NULL;
		lastTS = (u32)-1;
		lastShot = 0;
		itfs = NULL;

		memset(&cc, 0, sizeof(cc));
	}
};

static Bool CameraCalibration_GetNode(GF_Node *node, CameraCalibrationNode *rc)
{
	GF_FieldInfo field;
	memset(rc, 0, sizeof(CameraCalibrationNode));
	rc->sgprivate = node->sgprivate;
	u32 index = 0;

	/* source */
	if (gf_node_get_field(node, index++, &field) != GF_OK) return GF_FALSE;
	if (field.fieldType != GF_SG_VRML_MFSTRING) return GF_FALSE;
	rc->source = (MFString *) field.far_ptr;

	/* enabled */
	if (gf_node_get_field(node, index++, &field) != GF_OK) return GF_FALSE;
	if (field.fieldType != GF_SG_VRML_SFBOOL) return GF_FALSE;
	rc->enabled = (SFBool *) field.far_ptr;

	/* startTime */
	if (gf_node_get_field(node, index++, &field) != GF_OK) return GF_FALSE;
	if (field.fieldType != GF_SG_VRML_SFTIME) return GF_FALSE;
	rc->startTime = (SFTime *) field.far_ptr;

	/* timeBetweenSnapshots */
	if (gf_node_get_field(node, index++, &field) != GF_OK) return GF_FALSE;
	if (field.fieldType != GF_SG_VRML_SFTIME) return GF_FALSE;
	rc->timeBetweenSnapshots = (SFTime *) field.far_ptr;

	/* snapshotCount */
	if (gf_node_get_field(node, index++, &field) != GF_OK) return GF_FALSE;
	if (field.fieldType != GF_SG_VRML_SFINT32) return GF_FALSE;
	rc->snapshotCount = (SFInt32 *) field.far_ptr;

	/* boardSize */
	if (gf_node_get_field(node, index++, &field) != GF_OK) return GF_FALSE;
	if (field.fieldType != GF_SG_VRML_SFVEC2F) return GF_FALSE;
	rc->boardSize = (SFVec2f *) field.far_ptr;
	
	/* onStatus */
	if (gf_node_get_field(node, index++, &field) != GF_OK) return GF_FALSE;
	if (field.fieldType != GF_SG_VRML_SFINT32) return GF_FALSE;
	rc->onStatus = (SFInt32 *) field.far_ptr;

	return GF_TRUE;
}

static u32 DetectPatern(char* data, u32 size, u32 width, u32 height, u32 stride, u32 color, u32 boardx, u32 boardy, std::vector<cv::Point2f>& pointbuf )
{
	Bool draw_keypoints = GF_TRUE;
	Bool gpac_conv = GF_FALSE;
	IplImage* pImgOrig;

	IplImage* grayImage = cvCreateImage(cvSize(width, height), 8, 1);
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

				cvCvtColor(pImgOrig, grayImage, CV_RGB2GRAY );
				
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

				cvCvtColor(pImgOrig, grayImage, CV_RGBA2GRAY );
				
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

				cvCvtColor(pImgOrig, grayImage, CV_BGRA2GRAY );
				
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

				cvCvtColor(pImgOrig, grayImage, CV_BGR2GRAY );
				
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

				dst.width = width;
				dst.height = height;
				dst.pitch_x = 0;
				dst.pitch_y = dst.width * 3;
				dst.pixel_format = GF_PIXEL_BGR_24;
				dst.video_buffer = (char*)gf_malloc(dst.height * dst.pitch_y);

				wnd_src.x = 0;
				wnd_src.y = 0;
				wnd_src.w = width;
				wnd_src.h = height;

				wnd_dst.x = 0;
				wnd_dst.y = 0;
				wnd_dst.w = width;
				wnd_dst.h = height;

				gf_stretch_bits(&dst, &src, &wnd_dst, &wnd_src, 255, GF_FALSE, NULL, NULL);

				pImgOrig = cvCreateImageHeader(cvSize(width, height), 8, 3);
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

	cv::Size boardSize(boardx, boardy);
	bool found;
	cv::Mat img(pImgOrig);
	cv::Mat gray(grayImage);

	found = findChessboardCorners( gray, boardSize, pointbuf,
		CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

	if ( found )
		cornerSubPix( gray, pointbuf, cv::Size(11,11),
		cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

	if ( draw_keypoints )
	{
		if ( !gpac_conv && found )
		{
			cv::drawChessboardCorners( img, boardSize, cv::Mat(pointbuf), found );
			cvSetData(pImgOrig, NULL, 0);
			cvReleaseImage(&pImgOrig);
		}
	}

	return found ? 1 : 0;
}

enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

static double computeReprojectionErrors(
										const std::vector<std::vector<cv::Point3f> >& objectPoints,
										const std::vector<std::vector<cv::Point2f> >& imagePoints,
										const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
										const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
										std::vector<float>& perViewErrors )
{
	std::vector<cv::Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for( i = 0; i < (int)objectPoints.size(); i++ )
	{
		projectPoints(cv::Mat(objectPoints[i]), rvecs[i], tvecs[i],
			cameraMatrix, distCoeffs, imagePoints2);
		err = norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), CV_L2);
		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err*err/n);
		totalErr += err*err;
		totalPoints += n;
	}

	return std::sqrt(totalErr/totalPoints);
}

static void calcChessboardCorners(cv::Size boardSize, float squareSize, std::vector<cv::Point3f>& corners, Pattern patternType = CHESSBOARD)
{
	corners.resize(0);

	switch(patternType)
	{
	case CHESSBOARD:
	case CIRCLES_GRID:
		for( int i = 0; i < boardSize.height; i++ )
			for( int j = 0; j < boardSize.width; j++ )
				corners.push_back(cv::Point3f(float(j*squareSize),
				float(i*squareSize), 0));
		break;

	case ASYMMETRIC_CIRCLES_GRID:
		for( int i = 0; i < boardSize.height; i++ )
			for( int j = 0; j < boardSize.width; j++ )
				corners.push_back(cv::Point3f(float((2*j + i % 2)*squareSize),
				float(i*squareSize), 0));
		break;

	default:
		CV_Error(CV_StsBadArg, "Unknown pattern type\n");
	}
}

static bool runCalibration( std::vector<std::vector<cv::Point2f> > imagePoints,
						   cv::Size imageSize, cv::Size boardSize, Pattern patternType,
						   float squareSize, float aspectRatio,
						   int flags, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
						   std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs,
						   std::vector<float>& reprojErrs,
						   double& totalAvgErr)
{
	cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	if( flags & CV_CALIB_FIX_ASPECT_RATIO )
		cameraMatrix.at<double>(0,0) = aspectRatio;

	distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

	std::vector<std::vector<cv::Point3f> > objectPoints(1);
	calcChessboardCorners(boardSize, squareSize, objectPoints[0], patternType);

	objectPoints.resize(imagePoints.size(),objectPoints[0]);

	double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
		distCoeffs, rvecs, tvecs, flags|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
	///*|CV_CALIB_FIX_K3*/|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
	printf("RMS error reported by calibrateCamera: %g\n", rms);

	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
		rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

	return ok;
}

static bool runAndSave(GF_HardcodedProto* itfs, 
				const std::vector<std::vector<cv::Point2f> >& imagePoints,
				cv::Size imageSize, cv::Size boardSize, Pattern patternType, float squareSize,
				float aspectRatio, int flags, cv::Mat& cameraMatrix,
				cv::Mat& distCoeffs, bool writeExtrinsics, bool writePoints )
{
	std::vector<cv::Mat> rvecs, tvecs;
	std::vector<float> reprojErrs;
	double totalAvgErr = 0;

	bool ok = runCalibration(imagePoints, imageSize, boardSize, patternType, squareSize,
		aspectRatio, flags, cameraMatrix, distCoeffs,
		rvecs, tvecs, reprojErrs, totalAvgErr);
	printf("%s. avg reprojection error = %.2f\n",
		ok ? "Calibration succeeded" : "Calibration failed",
		totalAvgErr);

	if( ok ){
		char matrix[500];
		matrix[0] = 0;

		std::cout << cameraMatrix << std::endl;
		for ( int i = 0; i < cameraMatrix.rows; ++i )
			for ( int j = 0; j < cameraMatrix.cols; ++j )
				//std::cout << cameraMatrix.ptr<double>(i * cameraMatrix.cols + j);
				sprintf(matrix, "%s %0.5f", matrix, cameraMatrix.ptr<double>(i)[j]);

		std::cout<<std::endl;

		gf_modules_set_option ( ( GF_BaseInterface* ) itfs, "CameraCalibration", "Matrix", matrix );
	}
		
	return ok;
}

static void UpdateTextures(GF_TextureHandler *txh)
{
	u32 w, h, s, ar, pf;
	char* data;
	Bool eos;
	u32 ts, size;
	Bool images_loaded = GF_TRUE;
	Double currentTime;
	u32 status;

	GF_Node* node = txh->owner;
	CameraCalibrationStack *stack = (CameraCalibrationStack *)gf_node_get_private(node);

	CameraCalibration_GetNode(node, &(stack->cc));

	if ( !stack->loaded )
	{
		MFURL murl;
		SFURL surl;
		GF_Scene* scene = (GF_Scene*)gf_sg_get_private(gf_node_get_graph(node));
		surl.OD_ID = 0;

		murl.count = 1;
		murl.vals = &surl;
		surl.url = stack->cc.source->vals[0];
		stack->sourceStream = gf_scene_get_media_object_ex((GF_Scene*)gf_sg_get_private(gf_node_get_graph(node)), &murl, GF_MEDIA_OBJECT_VIDEO, GF_FALSE, NULL, GF_FALSE, node);

		if (stack->sourceStream)
		{
			stack->refreshTextureHandler.stream = stack->sourceStream;
			stack->loaded = GF_TRUE;
		}
	}

	if ( !stack->loaded )
		return;

	if ( stack->sourceStream->odm && stack->sourceStream->odm->state != GF_ODM_STATE_PLAY )
	{
		gf_mo_play(stack->sourceStream, 0, -1, GF_TRUE);
	}

	if ( stack->sourceStream->odm && stack->sourceStream->odm->codec )
	{
		if ( !(*(stack->cc.enabled))) 
			return;

		if ( stack->lastShot == *(stack->cc.snapshotCount) )
			return;

		currentTime = gf_node_get_scene_time(node);

		if ( currentTime < *(stack->cc.startTime) )
			return;

		Double diff = currentTime - *(stack->cc.startTime);

		if ( (diff / *(stack->cc.timeBetweenSnapshots)) <= stack->lastShot )
			return;

		/* Get video data*/
		gf_mo_get_visual_info(stack->sourceStream, &w, &h, &s, &ar, &pf, NULL);

		data = gf_mo_fetch_data(stack->sourceStream, GF_MO_FETCH_RESYNC, &eos, &ts, &size, NULL, NULL, NULL);

		if (!data || !size) {
			//gf_sc_invalidate(tr_state->visual->compositor, NULL);
			return;
		}

		stack->refreshTextureHandler.needs_release = GF_TRUE;

		if ( stack->lastTS == ts )
		{
			//gf_mo_release_data(stack->sourceStream, 0xFFFFFFFF, 1);
			return;
		}

		stack->lastTS = ts;

		GF_LOG(GF_LOG_WARNING, GF_LOG_MEDIA, ("[CameraCalibration] Video frame received: %dx%d, PTS = %d\n", w, h, ts));

		// Send video to engine

		int64 now, then;
		double elapsed_msec, tickspermsec=cvGetTickFrequency() * 1.0e3; 

		then = cvGetTickCount();

		std::vector<cv::Point2f> pointbuf;
		s32 found = DetectPatern(data, size, w, h, s, pf, FIX2FLT(stack->cc.boardSize->x), FIX2FLT(stack->cc.boardSize->y), pointbuf);

		now = cvGetTickCount();
		elapsed_msec = (now-then)/tickspermsec;
		GF_LOG(GF_LOG_WARNING, GF_LOG_MEDIA, ("[CameraCalibration] Detection time: %.0f\n", elapsed_msec));

		if ( found )
		{
			stack->lastShot++;
			stack->imagePoints.push_back(pointbuf);
			status = 1;
		}

		if ( stack->lastShot == *(stack->cc.snapshotCount) ){

			cv::Size imageSize(w, h);
			cv::Size boardSize(FIX2FLT(stack->cc.boardSize->x), FIX2FLT(stack->cc.boardSize->y));
			cv::Mat cameraMatrix, distCoeffs;
		
			if ( runAndSave(stack->itfs, stack->imagePoints, imageSize,
				boardSize, CHESSBOARD, 1.f, 1.f,
				0, cameraMatrix, distCoeffs,
				false, false))
	
				status = 2;
			else 
				status = -1;
		}
		
		//gf_mo_release_data(stack->sourceStream, 0xFFFFFFFF, 1);

		if ( status )
		{
			GF_FieldInfo onStatus;

			gf_node_get_field(node, 6, &onStatus);

			*(stack->cc.onStatus) = status;
			
			gf_node_event_out(node, onStatus.fieldIndex);
			gf_node_changed(node, &onStatus);
		}
	}

}

static void TraverseCameraCalibration(GF_Node *node, void *rs, Bool is_destroy)
{
	CameraCalibrationStack *stack = (CameraCalibrationStack *)gf_node_get_private(node);
	//GF_TraverseState *tr_state = (GF_TraverseState *) rs;


	if (is_destroy) {

		gf_sc_texture_destroy(&(stack->refreshTextureHandler));

		gf_mo_unregister(node, stack->sourceStream);
		stack->sourceStream = NULL;

		delete stack;

		gf_node_set_private(node, NULL);
		return;
	}
}

Bool compositor_init_camera_calibration(GF_HardcodedProto* itfs, GF_Compositor *compositor, GF_Node *node)
{
	CameraCalibrationNode cc;
	CameraCalibrationStack* stack;

	if (CameraCalibration_GetNode(node, &cc)) {

		if ( !cc.source->count )
			return GF_FALSE;

		stack = new CameraCalibrationStack();
		stack->itfs = itfs;

		CameraCalibration_GetNode(node, &(stack->cc));

		gf_node_set_private(node, stack);
		gf_node_set_callback_function(node, TraverseCameraCalibration);
		gf_node_proto_set_grouping(node);

		gf_sc_texture_setup(&(stack->refreshTextureHandler), compositor, node);
		stack->refreshTextureHandler.update_texture_fcnt = UpdateTextures;
		return GF_TRUE;
	}
	return GF_FALSE;
}
