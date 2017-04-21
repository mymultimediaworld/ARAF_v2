LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := Vuforia-prebuilt
LOCAL_SRC_FILES = ../../../../../../tools/vuforia-sdk-android-5-0-10/build/lib/$(TARGET_ARCH_ABI)/libVuforia.so
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/../../../../../../tools/vuforia-sdk-android-5-0-10/build/include
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)

OPENCV_CAMERA_MODULES:=off
OPENCV_LIB_TYPE:=SHARED
include ~/OpenCV-2.4.1/share/opencv/OpenCV.mk

LOCAL_MODULE		:= gm_loc_img

# Set OpenGL ES version-specific settings.
OPENGLES_LIB := -lGLESv2 
OPENGLES_DEF := -DUSE_OPENGL_ES_2_0

LOCAL_CFLAGS := -Wno-write-strings -Wno-psabi $(OPENGLES_DEF)

LOCAL_LDLIBS += -llog $(OPENGLES_LIB)

LOCAL_SHARED_LIBRARIES += Vuforia-prebuilt

include $(LOCAL_PATH)/base.mk

LOCAL_SRC_FILES := \
	../../../../modules/loc_img/loc_img.cpp \
	../../../../modules/loc_img/IDetector.cpp \
	../../../../modules/loc_img/FirstDetector.cpp \
	../../../../modules/loc_img/SecondDetector.cpp \
	../../../../modules/loc_img/Mcvsurf.cpp \
	../../../../modules/loc_img/ThirdDetector.cpp \
	../../../../modules/loc_img/surf.cpp \
	../../../../modules/loc_img/FourthDetector.cpp \
	../../../../modules/loc_img/camera_calibration.cpp \
	../../../../modules/loc_img/vuforia_camera.cpp \
	../../../../modules/loc_img/loc_img_vuforia.cpp

include $(BUILD_SHARED_LIBRARY)