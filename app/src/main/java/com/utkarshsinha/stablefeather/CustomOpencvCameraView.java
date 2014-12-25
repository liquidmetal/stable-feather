package com.utkarshsinha.stablefeather;

import android.content.Context;
import android.graphics.drawable.GradientDrawable;

import org.opencv.android.JavaCameraView;

/**
 * Created by utkarsh on 12/25/14.
 */
public class CustomOpencvCameraView extends JavaCameraView {
    CustomOpencvCameraView(Context ctx, int id) {
        super(ctx, id);
    }

    @Override
    protected boolean initializeCamera(int width, int height) {
        boolean output = super.initializeCamera(width, height);
        mCamera.setDisplayOrientation(90);
        return true;
    }
}