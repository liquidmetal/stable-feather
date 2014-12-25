package com.utkarshsinha.stablefeather;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.View;
import android.widget.Button;

/**
 * Created by utkarsh on 12/26/14.
 */
public class CameraOverlayWidget extends View {
    private Paint paint;
    private Button btn;

    public CameraOverlayWidget(Context ctx, AttributeSet attrs) {
        super(ctx, attrs);

        paint = new Paint();
        paint.setColor(Color.BLUE);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(3);
    }

    @Override
    public void onDraw(Canvas canvas) {
        canvas.drawRect(10, 10, canvas.getWidth()-20, canvas.getHeight()-20, paint);
    }
}
