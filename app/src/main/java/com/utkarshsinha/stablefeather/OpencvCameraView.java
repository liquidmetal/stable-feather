package com.utkarshsinha.stablefeather;

import android.content.Context;
import android.hardware.Camera;
import android.media.CamcorderProfile;
import android.media.MediaRecorder;
import android.util.Log;

import org.opencv.android.JavaCameraView;

import java.io.IOException;
import java.util.List;

/**
 * Created by utkarsh on 12/25/14.
 */
public class OpencvCameraView extends JavaCameraView {
    protected static final String TAG = "SFOCV::CameraView";
    protected MediaRecorder mMediaRecorder;

    OpencvCameraView(Context ctx, int id) {
        super(ctx, id);
    }

    @Override
    protected boolean initializeCamera(int width, int height) {
        boolean output = super.initializeCamera(width, height);

        tweakParameters();
        return output;
    }

    private void tweakParameters() {
        // Set the focus to infinity. We still don't know the focal length, though
        Camera.Parameters params = mCamera.getParameters();
        params.setFocusMode(Camera.Parameters.FOCUS_MODE_INFINITY);
        mCamera.setParameters(params);
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
    protected void prepareVideoRecorder() {
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
        mMediaRecorder.setVideoSize(mFrameHeight, mFrameWidth);
        mMediaRecorder.setVideoEncodingBitRate(profile.videoBitRate);
        mMediaRecorder.setVideoEncoder(profile.videoCodec);
    }

    /**
     * Starts recording video from the camera. It automatically creates a
     * media recorder object and saves the video to disk.
     * @param filename The file where the video will be stored.
     * @return if recording the video was successfully initiated or not.
     */
    public boolean recordVideo(String filename) {
        if(mMediaRecorder!=null) {
            // Something's already being recorded apparently
            return false;
        }

        prepareVideoRecorder();
        mMediaRecorder.setOutputFile(filename);

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

        // Start the recording!
        mMediaRecorder.start();
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

        return true;
    }

    protected void releaseMediaRecorder() {
        if(mMediaRecorder!=null) {
            mMediaRecorder.reset();
            mMediaRecorder.release();
            mMediaRecorder = null;
            mCamera.lock();
        }
    }
}