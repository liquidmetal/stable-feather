package com.utkarshsinha.stablefeather;

import com.utkarshsinha.stablefeather.util.SystemUiHider;

import android.annotation.TargetApi;
import android.app.Activity;
import android.content.pm.ActivityInfo;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.highgui.VideoCapture;
import org.opencv.imgproc.Imgproc;
import org.opencv.video.Video;

/**
 * An example full-screen activity that shows and hides the system UI (i.e.
 * status bar and navigation/system bar) with user interaction.
 *
 * @see SystemUiHider
 */
public class MainActivity extends Activity {
    private static String TAG = "SFOCV";
    private BaseLoaderCallback mOpencvLoadedCallback;
    private OpencvCameraView mOpencvCameraView;

    static {
        System.loadLibrary("stable_feather");
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        mOpencvLoadedCallback = new BaseLoaderCallback(this) {
            @Override
            public void onManagerConnected(int status) {
                switch(status) {
                    case LoaderCallbackInterface.SUCCESS:
                        Log.i(TAG, "OpenCV loaded successfully!");
                        mOpencvCameraView.enableView();
                        break;
                    default:
                        super.onManagerConnected(status);
                        break;
                }

            }
        };

        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
        mOpencvCameraView = new OpencvCameraView(this.getApplicationContext(), 0);
        setContentView(mOpencvCameraView);



        mOpencvCameraView.setCvCameraViewListener(new CameraBridgeViewBase.CvCameraViewListener() {
            @Override
            public void onCameraViewStarted(int width, int height) {

            }

            @Override
            public void onCameraViewStopped() {

            }

            @Override
            public Mat onCameraFrame(Mat inputFrame) {
                DrawCircle(50, inputFrame.getNativeObjAddr());
                return inputFrame;
            }
        });
        if(mOpencvCameraView==null) {
            Log.i(TAG, "Unable to find opencv_view...");
            this.finish();
        }

    }

    @Override
    protected void onPostCreate(Bundle savedInstanceState) {
        super.onPostCreate(savedInstanceState);
    }

    @Override
    public void onResume() {
        super.onResume();
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_10, this, mOpencvLoadedCallback);
    }

    @Override
    public void onPause() {
        super.onPause();
        if(mOpencvCameraView!=null)
            mOpencvCameraView.disableView();
    }

    public native boolean DrawCircle(long radius, long frameAddr);
}