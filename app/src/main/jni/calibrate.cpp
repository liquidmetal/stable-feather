#include <jni.h>

// Library includes
#include <android/log.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

// Local includes
#include "com_utkarshsinha_stablefeather_MainActivity.h"

#define  LOG_TAG    "SFOCV"

#define  LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define  LOGW(...)  __android_log_print(ANDROID_LOG_WARN, LOG_TAG, __VA_ARGS__)

using namespace std;

extern "C" {
JNIEXPORT jboolean JNICALL Java_com_utkarshsinha_stablefeather_MainActivity_DrawCircle
  (JNIEnv *, jobject, jlong lCircleSize, jlong matFrameAddr) {
      cv::Mat& matFrame = *(cv::Mat*)matFrameAddr;

      cv::circle(matFrame, cv::Point(100, 100), lCircleSize, cv::Scalar(255,0,255));
      return JNI_TRUE;
  }
}
