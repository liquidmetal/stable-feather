package com.utkarshsinha.stablefeather;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.media.CamcorderProfile;
import android.media.MediaRecorder;
import android.os.AsyncTask;
import android.os.Build;
import android.util.AttributeSet;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.ViewGroup;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.NativeCameraView;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.io.IOException;
import java.util.List;

/**
 * Created by utkarsh on 12/25/14.
 */
public class OpencvCameraView extends CameraBridgeViewBase implements Camera.PreviewCallback {
    private static final int MAGIC_TEXTURE_ID = 10;
    protected static final String TAG = "SFOCV::CameraView";

    private SurfaceTexture mSurfaceTexture;
    private byte mBuffer[];
    private Mat[] mFrameChain;
    private int mChainIdx = 0;
    private Thread mThread;
    private boolean mStopThread;

    protected Camera mCamera;
    protected JavaCameraFrame[] mCameraFrame;
    protected MediaRecorder mMediaRecorder;

    public OpencvCameraView(Context ctx, int id) {
        super(ctx, id);
    }
    public OpencvCameraView(Context ctx, AttributeSet attrs) {
        super(ctx, attrs);
    }

    public List<String> getEffectList() {
        return mCamera.getParameters().getSupportedColorEffects();
    }

    public boolean isEffectSupported() {
        return mCamera.getParameters().getColorEffect()!=null;
    }

    public String getEffect() {
        return mCamera.getParameters().getColorEffect();
    }

    public void setEffect(String effect) {
        Camera.Parameters params = mCamera.getParameters();
        params.setColorEffect(effect);
        mCamera.setParameters(params);
    }

    public List<Camera.Size> getResolutionList() {
        return mCamera.getParameters().getSupportedPreviewSizes();
    }

    /**
     * Sets up the appropriate objects to start recordingvideo
     */
    protected boolean prepareVideoRecorder() {
        CamcorderProfile profile = CamcorderProfile.get(CamcorderProfile.QUALITY_HIGH);
        profile.videoFrameHeight = mFrameHeight;
        profile.videoFrameWidth = mFrameWidth;

        mMediaRecorder = null;
        mMediaRecorder = new MediaRecorder();

        mCamera.unlock();
        mMediaRecorder.setCamera(mCamera);
        mMediaRecorder.setVideoSource(MediaRecorder.VideoSource.CAMERA);

        mMediaRecorder.setOutputFormat(profile.fileFormat);

        mMediaRecorder.setVideoFrameRate(profile.videoFrameRate);
        mMediaRecorder.setVideoSize(mFrameWidth, mFrameHeight);
        mMediaRecorder.setVideoEncodingBitRate(profile.videoBitRate);
        mMediaRecorder.setVideoEncoder(profile.videoCodec);

        mMediaRecorder.setOutputFile("/storage/emulated/0/Pictures/Recorder/testing.mp4");

        try {
            mMediaRecorder.prepare();
        }
        catch (IllegalStateException e) {
            Log.e(TAG, "IllegalStateException while preparing media recorder");
            return false;
        }
        catch (IOException e) {
            Log.e(TAG, "IOException while preparing media recorder");
            return false;
        }

        return true;
    }

    /**
     * Starts recording video from the camera. It automatically creates a
     * media recorder object and saves the video to disk.
     *
     * @param filename The file where the video will be stored.
     * @return if recording the video was successfully initiated or not.
     */
    public boolean recordVideo(String filename) {
        if(mMediaRecorder!=null) {
            // Something's already being recorded apparently
            return false;
        }

        new MediaPrepareTask().execute(null, null, null);

        return true;
    }

    public boolean stopRecord() {
        if(mMediaRecorder==null) {
            // Nothing's being recorded apparently
            return false;
        }

        // Stop recording, release everything and return true!
        mMediaRecorder.stop();
        releaseMediaRecorder();
        try {
            mCamera.reconnect();
        }
        catch (IOException e) {
            Log.e(TAG, "Unable to reconnect to the camera");
            return false;
        }
        //(mFrameWidth, mFrameHeight);
        //enableView();

        return true;
    }

    protected void releaseMediaRecorder() {
        if(mMediaRecorder!=null) {
            mMediaRecorder.reset();
            mMediaRecorder.release();
            mMediaRecorder = null;

            try {
                mCamera.reconnect();
                mCamera.lock();

                mCamera.addCallbackBuffer(mBuffer);
                mCamera.setPreviewCallbackWithBuffer(this);
                mCamera.setPreviewDisplay(getHolder());
                mCamera.startPreview();
            } catch (IOException e) {
                Log.e(TAG, "Unable to reconnect to camera");
            }
        }
    }

    protected boolean initializeCamera(int width, int height) {
        Log.d(TAG, "Initialize java camera");
        boolean result = true;
        synchronized (this) {
            mCamera = null;

            if (mCameraIndex == CAMERA_ID_ANY) {
                Log.d(TAG, "Trying to open camera with old open()");
                try {
                    mCamera = Camera.open();
                }
                catch (Exception e){
                    Log.e(TAG, "Camera is not available (in use or does not exist): " + e.getLocalizedMessage());
                }

                if(mCamera == null && Build.VERSION.SDK_INT >= Build.VERSION_CODES.GINGERBREAD) {
                    boolean connected = false;
                    for (int camIdx = 0; camIdx < Camera.getNumberOfCameras(); ++camIdx) {
                        Log.d(TAG, "Trying to open camera with new open(" + Integer.valueOf(camIdx) + ")");
                        try {
                            mCamera = Camera.open(camIdx);
                            connected = true;
                        } catch (RuntimeException e) {
                            Log.e(TAG, "Camera #" + camIdx + "failed to open: " + e.getLocalizedMessage());
                        }
                        if (connected) break;
                    }
                }
            } else {
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.GINGERBREAD) {
                    int localCameraIndex = mCameraIndex;
                    if (mCameraIndex == CAMERA_ID_BACK) {
                        Log.i(TAG, "Trying to open back camera");
                        Camera.CameraInfo cameraInfo = new Camera.CameraInfo();
                        for (int camIdx = 0; camIdx < Camera.getNumberOfCameras(); ++camIdx) {
                            Camera.getCameraInfo( camIdx, cameraInfo );
                            if (cameraInfo.facing == Camera.CameraInfo.CAMERA_FACING_BACK) {
                                localCameraIndex = camIdx;
                                break;
                            }
                        }
                    } else if (mCameraIndex == CAMERA_ID_FRONT) {
                        Log.i(TAG, "Trying to open front camera");
                        Camera.CameraInfo cameraInfo = new Camera.CameraInfo();
                        for (int camIdx = 0; camIdx < Camera.getNumberOfCameras(); ++camIdx) {
                            Camera.getCameraInfo( camIdx, cameraInfo );
                            if (cameraInfo.facing == Camera.CameraInfo.CAMERA_FACING_FRONT) {
                                localCameraIndex = camIdx;
                                break;
                            }
                        }
                    }
                    if (localCameraIndex == CAMERA_ID_BACK) {
                        Log.e(TAG, "Back camera not found!");
                    } else if (localCameraIndex == CAMERA_ID_FRONT) {
                        Log.e(TAG, "Front camera not found!");
                    } else {
                        Log.d(TAG, "Trying to open camera with new open(" + Integer.valueOf(localCameraIndex) + ")");
                        try {
                            mCamera = Camera.open(localCameraIndex);
                        } catch (RuntimeException e) {
                            Log.e(TAG, "Camera #" + localCameraIndex + "failed to open: " + e.getLocalizedMessage());
                        }
                    }
                }
            }

            if (mCamera == null)
                return false;

            /* Now set camera parameters */
            try {
                Camera.Parameters params = mCamera.getParameters();
                Log.d(TAG, "getSupportedPreviewSizes()");
                List<android.hardware.Camera.Size> sizes = params.getSupportedVideoSizes();

                if (sizes != null) {
                    /* Select the size that fits surface considering maximum size allowed */
                    //Size frameSize = calculateCameraFrameSize(sizes, new JavaCameraSizeAccessor(), width, height);
                    Camera.Size frameSize = getOptimalPreviewSize(sizes, width, height);

                    params.setPreviewFormat(ImageFormat.NV21);
                    Log.d(TAG, "Set preview size to " + Integer.valueOf((int)frameSize.width) + "x" + Integer.valueOf((int)frameSize.height));
                    params.setPreviewSize((int)frameSize.width, (int)frameSize.height);

                    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.ICE_CREAM_SANDWICH && !android.os.Build.MODEL.equals("GT-I9100"))
                        params.setRecordingHint(true);

                    // [utkarsh] Making this infinity focus instead of video continuous
                    List<String> FocusModes = params.getSupportedFocusModes();
                    if (FocusModes != null && FocusModes.contains(Camera.Parameters.FOCUS_MODE_INFINITY)) {
                        params.setFocusMode(Camera.Parameters.FOCUS_MODE_INFINITY);
                    }

                    mCamera.setParameters(params);
                    params = mCamera.getParameters();

                    mFrameWidth = params.getPreviewSize().width;
                    mFrameHeight = params.getPreviewSize().height;

                    if ((getLayoutParams().width == ViewGroup.LayoutParams.MATCH_PARENT) && (getLayoutParams().height == ViewGroup.LayoutParams.MATCH_PARENT))
                        mScale = Math.min(((float)height)/mFrameHeight, ((float)width)/mFrameWidth);
                    else
                        mScale = 0;

                    if (mFpsMeter != null) {
                        mFpsMeter.setResolution(mFrameWidth, mFrameHeight);
                    }

                    int size = mFrameWidth * mFrameHeight;
                    size  = size * ImageFormat.getBitsPerPixel(params.getPreviewFormat()) / 8;
                    mBuffer = new byte[size];

                    // [utkarsh] Disabling these callbacks essentially disables the hooks OpenCV
                    // has
                    mCamera.addCallbackBuffer(mBuffer);
                    mCamera.setPreviewCallbackWithBuffer(this);

                    mFrameChain = new Mat[2];
                    mFrameChain[0] = new Mat(mFrameHeight + (mFrameHeight/2), mFrameWidth, CvType.CV_8UC1);
                    mFrameChain[1] = new Mat(mFrameHeight + (mFrameHeight/2), mFrameWidth, CvType.CV_8UC1);

                    AllocateCache();

                    mCameraFrame = new JavaCameraFrame[2];
                    mCameraFrame[0] = new JavaCameraFrame(mFrameChain[0], mFrameWidth, mFrameHeight);
                    mCameraFrame[1] = new JavaCameraFrame(mFrameChain[1], mFrameWidth, mFrameHeight);

                    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB) {
                        mSurfaceTexture = new SurfaceTexture(MAGIC_TEXTURE_ID);
                        mCamera.setPreviewTexture(mSurfaceTexture);
                        mCamera.setPreviewDisplay(getHolder());

                    } else
                        mCamera.setPreviewDisplay(null);

                    /* Finally we are ready to start the preview */
                    Log.d(TAG, "startPreview");
                    mCamera.startPreview();
                }
                else
                    result = false;

            } catch (Exception e) {
                result = false;
                e.printStackTrace();
            }
        }

        return result;
    }

    protected void releaseCamera() {
        synchronized (this) {
            if (mCamera != null) {
                mCamera.stopPreview();
                mCamera.setPreviewCallback(null);

                mCamera.release();
            }
            // [utkarsh] Just testing this out!
            mCamera = null;
            if (mFrameChain != null) {
                mFrameChain[0].release();
                mFrameChain[1].release();
            }
            if (mCameraFrame != null) {
                mCameraFrame[0].release();
                mCameraFrame[1].release();
            }
        }
    }

    @Override
    protected boolean connectCamera(int width, int height) {

        /* 1. We need to instantiate camera
         * 2. We need to start thread which will be getting frames
         */
        /* First step - initialize camera connection */
        Log.d(TAG, "Connecting to camera");
        if (!initializeCamera(width, height))
            return false;

        /* now we can start update thread */
        Log.d(TAG, "Starting processing thread");
        mStopThread = false;
        mThread = new Thread(new CameraWorker());
        mThread.start();

        return true;
    }

    protected void disconnectCamera() {
        /* 1. We need to stop thread which updating the frames
         * 2. Stop camera and release it
         */
        Log.d(TAG, "Disconnecting from camera");
        try {
            mStopThread = true;
            Log.d(TAG, "Notify thread");
            synchronized (this) {
                this.notify();
            }
            Log.d(TAG, "Wating for thread");
            if (mThread != null)
                mThread.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        } finally {
            mThread =  null;
        }

        /* Now release camera */
        releaseCamera();
    }

    /**
     * Called whenever a preview frame is made available. This is where a
     * Mat is created out of the data received from the camera.
     *
     * @param frame The raw pixel data received from the camera
     * @param cam The camera which returned the data
     */
    public void onPreviewFrame(byte[] frame, Camera cam) {
        synchronized (this) {
            mFrameChain[1 - mChainIdx].put(0, 0, frame);
            this.notify();
        }
        if (mCamera != null)
            mCamera.addCallbackBuffer(mBuffer);
    }

    @Override
    public void surfaceChanged(SurfaceHolder arg0, int arg1, int arg2, int arg3) {
        try {
            if (mCamera != null && mBuffer != null) {
                // The surface was changed and the camera exists - so need to
                // register the callback again
                // http://stackoverflow.com/questions/6386903/android-preview-processing-while-video-recording
                mCamera.addCallbackBuffer(mBuffer);
                mCamera.setPreviewCallbackWithBuffer(this);
                mCamera.setPreviewDisplay(arg0);
                mCamera.startPreview();
                Log.d(TAG, "Surface changed, registered again");
            }
        } catch(IOException e) {
            Log.e(TAG, "Unable to change surfage here");
        }

        super.surfaceChanged(arg0, arg1, arg2, arg3);
    }

    /**
     * Given a list of supported image sizes, finds the closest match to the given
     * width and height requirements
     *
     * @param sizes A list of all sizes supported by the camera
     * @param w The desired width of the image
     * @param h The desired height of the image
     * @return the closest match to the given width and height.
     */
    protected Camera.Size getOptimalPreviewSize(List<Camera.Size> sizes, int w, int h) {
        final double ASPECT_TOLERANCE = 0.1;
        double targetRatio = (double)w / h;

        if(sizes == null) {
            return null;
        }

        Camera.Size optimalSize = null;

        double minDiff = Double.MAX_VALUE;

        int targetHeight = h;

        for (Camera.Size size : sizes) {
            double ratio = (double)size.width / size.height;
            double diff = Math.abs(ratio - targetRatio);

            if(Math.abs(ratio - targetRatio) > ASPECT_TOLERANCE)
                continue;

            if(Math.abs(size.height - targetHeight) < minDiff) {
                optimalSize = size;
                minDiff = Math.abs(size.height - targetHeight);
            }
        }

        if(optimalSize == null) {
            minDiff = Double.MAX_VALUE;
            for(Camera.Size size : sizes) {
                if(Math.abs(size.height - targetHeight) < minDiff) {
                    optimalSize = size;
                    minDiff = Math.abs(size.height - targetHeight);
                }
            }
        }

        return optimalSize;
    }

    public void stopPreview() {
        if(mCamera!=null) {
            mCamera.stopPreview();

            try {
                mCamera.setPreviewDisplay(null);
            }
            catch(IOException e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * Starts preview after a stopPreview call.
     * TODO Identify why this never works
     */
    public void startPreview() {
        if(mCamera!=null) {
            Log.d(TAG, "Trying to restart preview");

            try {
                mCamera.setPreviewDisplay(getHolder());
            }
            catch(IOException e) {
                e.printStackTrace();
            }

            mCamera.startPreview();
        }
    }

    /**
     * Lets you set a specific preview image
     * @param frame The image to set
     */
    public void setPreviewFrame(Mat frame) {
        if(frame == null)
            // Nothing to do here
            return;

        Bitmap mCacheBitmap = Bitmap.createBitmap(mFrameWidth, mFrameHeight, Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(frame, mCacheBitmap);

        Canvas canvas = getHolder().lockCanvas();
        if (canvas != null) {
            Log.d(TAG, "Setting preview to a custom frame!");
            canvas.drawColor(0, android.graphics.PorterDuff.Mode.CLEAR);

            canvas.drawBitmap(mCacheBitmap, new Rect(0,0,mCacheBitmap.getWidth(), mCacheBitmap.getHeight()),
                    new Rect((canvas.getWidth() - mCacheBitmap.getWidth()) / 2,
                            (canvas.getHeight() - mCacheBitmap.getHeight()) / 2,
                            (canvas.getWidth() - mCacheBitmap.getWidth()) / 2 + mCacheBitmap.getWidth(),
                            (canvas.getHeight() - mCacheBitmap.getHeight()) / 2 + mCacheBitmap.getHeight()), null);

            getHolder().unlockCanvasAndPost(canvas);
        }
    }

    public float getFocalLength() {
        Camera.Parameters params = mCamera.getParameters();
        return params.getFocalLength();
    }

    private class JavaCameraFrame implements CvCameraViewFrame {
        public Mat gray() {
            return mYuvFrameData.submat(0, mHeight, 0, mWidth);
        }

        public Mat rgba() {
            Imgproc.cvtColor(mYuvFrameData, mRgba, Imgproc.COLOR_YUV2RGBA_NV21, 4);
            return mRgba;
        }

        public JavaCameraFrame(Mat Yuv420sp, int width, int height) {
            super();
            mWidth = width;
            mHeight = height;
            mYuvFrameData = Yuv420sp;
            mRgba = new Mat();
        }

        public void release() {
            mRgba.release();
        }

        private Mat mYuvFrameData;
        private Mat mRgba;
        private int mWidth;
        private int mHeight;
    };

    private class CameraWorker implements Runnable {
        public void run() {
            do {
                synchronized (OpencvCameraView.this) {
                    try {
                        OpencvCameraView.this.wait();
                    } catch (InterruptedException e) {
                        // TODO Auto-generated catch block
                        e.printStackTrace();
                    }
                }

                if (!mStopThread) {
                    if (!mFrameChain[mChainIdx].empty()) {
                        // Simply receive the frame - we won't be modifying the image yet
                        receivePreviewFrame(mCameraFrame[mChainIdx]);

                        // [utkarsh] Disabled so I don't end up drawing the frame twice
                        // Also, it causes troubles later on
                        // deliverAndDrawFrame(mCameraFrame[mChainIdx]);
                    }
                    mChainIdx = 1 - mChainIdx;
                }
            } while (!mStopThread);
            Log.d(TAG, "Finish processing thread");
        }
    }

    protected void receivePreviewFrame(CvCameraViewFrame frame) {
    }

    public static class JavaCameraSizeAccessor implements ListItemAccessor {

        public int getWidth(Object obj) {
            Camera.Size size = (Camera.Size) obj;
            return size.width;
        }

        public int getHeight(Object obj) {
            Camera.Size size = (Camera.Size) obj;
            return size.height;
        }
    }

    class MediaPrepareTask extends AsyncTask<Void, Void, Boolean> {
        @Override
        protected Boolean doInBackground(Void... voids) {
            if(prepareVideoRecorder()) {
                mMediaRecorder.start();

                try {
                    mCamera.reconnect();
                }
                catch (IOException e) {
                    Log.e(TAG, "IOException when trying to reconnect after unlocking");
                    return false;
                }
            } else {
                releaseMediaRecorder();
                return false;
            }
            return true;
        }

        @Override
        protected void onPostExecute(Boolean result) {
            if (!result) {
                Log.i(TAG, "Something failed during the MediaPrepareTask");
            }
        }
    }
}