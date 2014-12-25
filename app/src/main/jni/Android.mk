LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

# As mentioned here: http://stackoverflow.com/questions/24669518/non-system-libraries-in-linker-flags
override OPENCV_INSTALL_MODULES:=on

include /work/lib/opencv/OpenCV-2.4.10-android-sdk/sdk/native/jni/OpenCV.mk

LOCAL_MODULE := stable_feather
LOCAL_SRC_FILES := calibrate.cpp
LOCAL_LDLIBS += -llog -ldl

include $(BUILD_SHARED_LIBRARY)
