package com.utkarshsinha.stablefeather;

import android.content.Context;
import android.graphics.drawable.GradientDrawable;
import android.hardware.Camera;

import org.opencv.android.JavaCameraView;

import java.util.List;

/**
 * Created by utkarsh on 12/25/14.
 */
public class OpencvCameraView extends JavaCameraView {
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
        // So - we need a fixed focal length.
        Camera.Parameters params = mCamera.getParameters();
        params.setFocusMode(Camera.Parameters.FOCUS_MODE_FIXED);
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


}