#include <jni.h>

// Library includes
#include <android/log.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

// Local includes
#include "com_utkarshsinha_stablefeather_MainActivity.h"

#define  LOG_TAG    "SFOCV"

#define  LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define  LOGW(...)  __android_log_print(ANDROID_LOG_WARN, LOG_TAG, __VA_ARGS__)

using namespace std;
using namespace cv;

extern "C" {
JNIEXPORT jboolean JNICALL Java_com_utkarshsinha_stablefeather_MainActivity_JustSomething
  (JNIEnv *, jobject, jlong lCircleSize, jobject matFrame) {
      LOGI("Hey man, I'm in the native code!");
      return JNI_TRUE;
  }
}
