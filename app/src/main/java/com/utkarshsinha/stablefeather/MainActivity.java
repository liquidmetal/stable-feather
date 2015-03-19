package com.utkarshsinha.stablefeather;

import com.utkarshsinha.stablefeather.util.SystemUiHider;

import android.annotation.TargetApi;
import android.app.ActionBar;
import android.app.Activity;
import android.app.ProgressDialog;
import android.content.Context;
import android.content.pm.ActivityInfo;
import android.graphics.Matrix;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.SystemClock;
import android.util.Log;
import android.view.GestureDetector;
import android.view.Gravity;
import android.view.HapticFeedbackConstants;
import android.view.MotionEvent;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.FrameLayout;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.core.Point3;
import org.opencv.highgui.VideoCapture;
import org.opencv.imgproc.Imgproc;
import org.opencv.video.Video;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * An example full-screen activity that shows and hides the system UI (i.e.
 * status bar and navigation/system bar) with user interaction.
 *
 * @see SystemUiHider
 */
public class MainActivity extends Activity implements SensorEventListener, CameraOverlayWidget.OverlayEventListener {
    private static String TAG = "SFOCV";
    private BaseLoaderCallback mOpencvLoadedCallback;
    private OpencvCameraView mOpencvCameraView;

    private CameraOverlayWidget mCameraOverlay;
    private GestureDetector.SimpleOnGestureListener mOverlayGestureDetector = null;

    private FrameLayout mFrameLayout;

    private SensorManager mSensorManager;
    private Sensor mGyro;
    private Sensor mRotation = null;

    private Map<Long, Point3> gyroCache = null;
    private Map<Long, Point3> driftCache = null;
    private Long mStartTime = -1L;
    private Long mLastTime = -1L;

    private Map<Long, Point3> thetaCache = null;
    private Long mThetaStartTime = -1L;
    private Long mThetaLastTime = -1L;

    protected boolean isRecording = false;

    private EstimatedCameraParameters estimateParams = null;

    static {
        System.loadLibrary("stable_feather");
    }



    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        requestWindowFeature(Window.FEATURE_NO_TITLE);
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_main);

        mFrameLayout = (FrameLayout)findViewById(R.id.frameLayout);
        mOpencvCameraView = new OpencvCameraView(getApplicationContext(), 0);
        mCameraOverlay = new CameraOverlayWidget(getApplicationContext(), null);

        gyroCache = new ConcurrentHashMap<Long, Point3>();
        thetaCache = new ConcurrentHashMap<Long, Point3>();
        driftCache = new ConcurrentHashMap<Long, Point3>();

        setupSensors();

        // mCameraOverlay.setCustomTouchMethods(mOverlayGestureDetector);
        mCameraOverlay.setOverlayEventListener(this);

        mFrameLayout.addView(mOpencvCameraView, new FrameLayout.LayoutParams(FrameLayout.LayoutParams.MATCH_PARENT, FrameLayout.LayoutParams.MATCH_PARENT, Gravity.CENTER));
        mFrameLayout.addView(mCameraOverlay, new FrameLayout.LayoutParams(FrameLayout.LayoutParams.MATCH_PARENT, FrameLayout.LayoutParams.MATCH_PARENT, Gravity.CENTER));

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


    /**
     * This method receives raw sensor input from the gyroscope. Store it whenever
     * a video is being recorded
     *
     * @param sensorEvent Information from the sensor
     * @returns nothing
     */
    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        if(mStartTime == -1)
            mStartTime = sensorEvent.timestamp;

        if(mThetaStartTime == -1)
            mThetaStartTime = sensorEvent.timestamp;

        if(sensorEvent.sensor.getType() == Sensor.TYPE_GYROSCOPE_UNCALIBRATED) {
            Point3 omega = new Point3(sensorEvent.values[0], sensorEvent.values[1], sensorEvent.values[2]);
            Point3 drift = new Point3(sensorEvent.values[3], sensorEvent.values[4], sensorEvent.values[5]);

            gyroCache.put(sensorEvent.timestamp - mStartTime, omega);
            driftCache.put(sensorEvent.timestamp - mStartTime, drift);
            mLastTime = sensorEvent.timestamp;

            mCameraOverlay.setOmega(omega);
            mCameraOverlay.setDrift(drift);
        } else if(sensorEvent.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
            float[] mRotationMatrix = new float[9];
            float[] thetaArray = new float[3];

            SensorManager.getRotationMatrixFromVector(mRotationMatrix, sensorEvent.values);
            SensorManager.remapCoordinateSystem(mRotationMatrix, SensorManager.AXIS_X, SensorManager.AXIS_Y, thetaArray);
            SensorManager.getOrientation(mRotationMatrix, thetaArray);

            Point3 theta = new Point3(thetaArray[0], thetaArray[1], thetaArray[2]);
            thetaCache.put(sensorEvent.timestamp - mThetaStartTime, theta);
            mThetaLastTime = sensorEvent.timestamp;
            mCameraOverlay.setTheta(theta);
        }
    }

    /**
     * Accuracy changed? This event notifies you of that.
     * @param sensor The sensor for which the accuracy changed
     * @param i The new accuracy
     */
    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }

    /**
     * Setup the sensor infrastructure
     * @returns nothing
     */
    private void setupSensors() {
        mSensorManager = (SensorManager)getSystemService(Context.SENSOR_SERVICE);
        mGyro = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED);
        mRotation = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);

        mSensorManager.registerListener(this, mGyro, SensorManager.SENSOR_DELAY_FASTEST);
        mSensorManager.registerListener(this, mRotation, SensorManager.SENSOR_DELAY_FASTEST);
    }

    @Override
    protected void onPostCreate(Bundle savedInstanceState) {
        super.onPostCreate(savedInstanceState);
    }

    @Override
    public void onResume() {
        super.onResume();
        Log.d(TAG, "Resuming the main activity");
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_10, this, mOpencvLoadedCallback);

        mCameraOverlay.resume();
    }

    @Override
    public void onPause() {
        super.onPause();

        if(mOpencvCameraView!=null)
            mOpencvCameraView.disableView();

        mCameraOverlay.pause();
    }

    private File getOutputMediaFile() {
        if(!Environment.getExternalStorageState().equalsIgnoreCase(Environment.MEDIA_MOUNTED)) {
            return null;
        }

        File mediaStorageDir = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES), "Recorder");

        if(!mediaStorageDir.exists()) {
            if(!mediaStorageDir.mkdirs()) {
                Log.d("Recorder", "Failed to create directory");
                return null;
            }
        }

        String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
        File mediaFile;
        mediaFile = new File(mediaStorageDir.getPath() + File.separator + "VID_" + timeStamp + ".mp4");

        return mediaFile;
    }

    public native boolean DrawCircle(long radius, long frameAddr);
    public native boolean ReadVideo(String file);

    @Override
    public boolean onCalibrate(MotionEvent event) {
        if(!isRecording) {
            // Notify the view
            mCameraOverlay.setRecording();

            resetGyroCache();

            // Do the job
            String output = getOutputMediaFile().toString();
            Log.d(TAG, "Output file: " + output);
            mOpencvCameraView.recordVideo(output);

            mOpencvCameraView.performHapticFeedback(HapticFeedbackConstants.VIRTUAL_KEY);

            isRecording = true;
        } else {
            // Do the job
            mOpencvCameraView.stopRecord();
            mOpencvCameraView.performHapticFeedback(HapticFeedbackConstants.VIRTUAL_KEY);

            // Notify the view
            mCameraOverlay.unsetRecording();
            mOpencvCameraView.stopPreview();

            CameraCalibrator cc = new CameraCalibrator(getOutputMediaFile().getParent() + "/testing.mp4", gyroCache, driftCache, thetaCache, mOpencvCameraView);
            cc.mProgressListener = new CameraCalibrator.CalibrationProgressListener() {
                @Override
                public boolean onFinish(EstimatedCameraParameters params, Mat alignmentGraph) {
                    // TODO Store the estimated camera parameters here!
                    resetGyroCache();
                    mOpencvCameraView.setPreviewFrame(alignmentGraph);
                    // mOpencvCameraView.startPreview();
                    estimateParams = params;
                    Log.d(TAG, "Started preview from camera after doing estimate");
                    return false;
                }

                @Override
                public boolean onProgress(long frame) {
                    return false;
                }
            };
            cc.startCalibration();

            isRecording = false;
        }
        return true;
    }

    protected void resetGyroCache() {
        // Setup our gyroscope cache!
        gyroCache.clear();
        driftCache.clear();
        mStartTime = -1L;
        mLastTime = -1L;

        thetaCache.clear();
        mThetaStartTime = -1L;
        mLastTime = -1L;
    }
}
