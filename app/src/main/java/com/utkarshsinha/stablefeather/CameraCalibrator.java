package com.utkarshsinha.stablefeather;

import android.hardware.SensorManager;
import android.os.Environment;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.features2d.DMatch;
import org.opencv.features2d.DescriptorExtractor;
import org.opencv.features2d.DescriptorMatcher;
import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.KeyPoint;
import org.opencv.imgproc.Imgproc;

import java.io.BufferedOutputStream;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Created by utkarsh on 12/27/14.
 */
public class CameraCalibrator implements CalibrateWorker.CalibrateWorkerProgressListener {

    private String mFilename;
    private CalibrateWorker mThread;
    protected CalibrationProgressListener mProgressListener;
    protected Map<Long, Point3> gyroSamples = null;
    protected Map<Long, Point3> driftSamples = null;

    private static final String TAG = "SFOCV::CameraCalibrator";

    public CameraCalibrator(String filename, Map<Long, Point3> gyro, Map<Long, Point3> drift, OpencvCameraView cameraView) {
        mFilename = filename;
        gyroSamples = gyro;
        driftSamples = drift;

        mThread = new CalibrateWorker();
        mThread.setFilename(mFilename);
        mThread.setProgressListener(this);
        mThread.setCameraView(cameraView);
        mThread.setGyroSamples(gyroSamples, driftSamples);
        // mThread.setThetaSamples(thetaSamples);
        mThread.setHandler(new Handler(Looper.getMainLooper()));
    }

    public void setProgressListener(CalibrationProgressListener listener) {
        mProgressListener = listener;
    }

    public void startCalibration() {
        mThread.start();
    }

    public void waitForCalibration() {
        try {
            mThread.join();
        }
        catch(InterruptedException e) {
            e.printStackTrace();
        }
    }

    /**
     * Called from the thread whenever there's an update
     * @return
     */
    @Override
    public boolean onProgress(long frame) {
        // Propagate the message
        if(mProgressListener!=null)
            return mProgressListener.onProgress(frame);

        return false;
    }

    @Override
    public boolean onFinish(EstimatedCameraParameters params, Mat alignmentGraph) {
        if(mProgressListener!=null)
            return mProgressListener.onFinish(params, alignmentGraph);

        return false;
    }

    public interface CalibrationProgressListener {
        public boolean onProgress(long frame);
        public boolean onFinish(EstimatedCameraParameters params, Mat alignmentGraph);
    }
}

/**
 * CalibrateWorks does the actual work of running the calibration. It is designed to started in
 * a separate thread to keep the application responsive!
 */
class CalibrateWorker extends Thread implements SequentialFrameExtractor.FrameAvailableListener {
    private CalibrateWorkerProgressListener mProgressListener = null;
    private Handler mMainHandler = null;
    private OpencvCameraView mCameraView = null;

    private Mat alignmentGraph = null;
    private Mat previewMat = null;
    private Frame previewFrame = null;
    private String mFilename;
    private static final String TAG = "SFOCV::CalibrateWorker";
    private List<Frame> allFrames = null;

    int permutation = 0;

    private LinkedList<Long> sortedTimestamps = null;
    private Map<Long, Point3> gyroSamples = null;

    private LinkedList<Long> sortedThetaTimestamps = null;
    private Map<Long, Point3> thetaSamples = null;
    private Map<Long, Point3> driftSamples = null;

    private Frame previousFrame = null;
    private KeyPoint[] previousKeypoints = null;
    private Mat previousKeypointDescriptors = null;

    private EstimatedCameraParameters estimateParams = null;
    protected long mFrameWidth = 0;
    protected long mFrameHeight = 0;

    static {
        System.loadLibrary("stable_feather");
    }

    public void setProgressListener(CalibrateWorkerProgressListener listener) {
        mProgressListener = listener;
    }
    public void setHandler(Handler h) {
        mMainHandler = h;
    }
    public void setFilename(String mFilename) {
        this.mFilename = mFilename;
    }
    public void setCameraView(OpencvCameraView cameraView) {
        mCameraView = cameraView;
    }

    /**
     * Saves all data into a file
     * TODO remove this soon
     */
    private void saveDataToFile() throws IOException {
        int numFrames = allFrames.size();
        int numGyroSamples = sortedTimestamps.size();
        int numThetaSamples = sortedThetaTimestamps.size();

        File dataStorageDir = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES), "Recorder");
        File dataFile = new File(dataStorageDir, "data.csv");

        if(dataFile.exists())
            dataFile.delete();

        dataFile.createNewFile();
        FileWriter fw = new FileWriter(dataFile.getAbsolutePath());
        BufferedWriter bw = new BufferedWriter(fw);

        // Write out the gyro samples
        bw.write(numGyroSamples+"\n");
        for(int i=0;i<numGyroSamples;i++) {
            try {
                long timestamp = sortedTimestamps.get(i);
                Point3 omega = gyroSamples.get(timestamp);
                Point3 drift = driftSamples.get(timestamp);

                bw.write(timestamp + "," + omega.x + "," + omega.y + "," + omega.z + "," + drift.x + "," + drift.y + "," + drift.z + "\n");
            } catch(Exception e) {
                e.printStackTrace();
            }
        }

        // Write out the theta estimates
        assert(sortedThetaTimestamps.size() == thetaSamples.size());
        bw.write(numThetaSamples+"\n");
        for(int i=0;i<numThetaSamples;i++) {
            try {
                long timestamp = sortedThetaTimestamps.get(i);
                Point3 theta = thetaSamples.get(timestamp);

                bw.write(timestamp + "," + theta.x + "," + theta.y + "," + theta.z + "\n");
            } catch(Exception e) {
                e.printStackTrace();
            }
        }

        // Write out the actual frames
        bw.write(numFrames+"\n");
        for(int i=0;i<numFrames;i++) {
            try {
                // The first frame is special - it doesn't have previous/currentKeypoints
                if(i == 0) {
                    Frame f = allFrames.get(i);
                    bw.write(f.timestamp + ",0" + "\n");
                    continue;
                }

                Frame f = allFrames.get(i);
                int numKeypoints = f.currentKeypoints.size();

                bw.write(f.timestamp + "," + numKeypoints + "\n");

                for (int j = 0; j < numKeypoints; j++) {
                    KeyPoint ptCurrent = f.currentKeypoints.get(j);
                    KeyPoint ptPrevious = f.previousKeypoints.get(j);

                    bw.write(ptPrevious.pt.x + "," + ptPrevious.pt.y + "," + ptCurrent.pt.x + "," + ptCurrent.pt.y + "\n");
                }
            } catch(NullPointerException e) {
                e.printStackTrace();
            }
        }

        bw.flush();
        bw.close();

        Log.i(TAG, "Done writing the keypoints to the file");
    }

    @Override
    public void run() {
        super.run();

        estimateParams = new EstimatedCameraParameters();
        allFrames = new LinkedList<Frame>();

        // This will trigger onFrameAvailable() - stepping through each frame
        SequentialFrameExtractor frameExtractor = new SequentialFrameExtractor(mFilename);
        frameExtractor.setFrameAvailableListener(this);
        frameExtractor.start();

        // Now that we have the angular velocities, calculate the theta
        integrateAngularVelocity(allFrames);

        mFrameWidth = previewMat.cols();
        mFrameHeight = previewMat.rows();

        try {
            Log.i(TAG, "Trying to write data to file");
            saveDataToFile();
        } catch(IOException e) {
            Log.e(TAG, "Error writing data to file");
            e.printStackTrace();
        }

        // At this point, we have the allFrames populated. Now we can start estimating parameters.
        for(int i=2;i<6;i++) {
            permutation = i;
            Log.i(TAG, "TRYING PERMUTATION " + permutation);
            calculateEstimate();
        }

        // Now we have all the parameters - we try to generate a graph of the estimated parameters
        alignmentGraph = generateAlignmentGraph();

        // Trigger the finish event
        if(mProgressListener!=null) {
            // Should run
            mMainHandler.post(new Runnable() {
                @Override
                public void run() {
                    mProgressListener.onFinish(estimateParams, alignmentGraph);
                }
            });
        }
    }

    protected Mat generateAlignmentGraph() {
        Mat ret = Mat.zeros((int)mFrameHeight, (int)mFrameWidth, CvType.CV_8UC4);

        ret.setTo(new Scalar(255, 255, 255, 255));

        // Draw the per-frame translation
        int numFrames = allFrames.size();
        LinkedList<Long> translVideo = new LinkedList<Long>();
        long translVideoMax = Long.MIN_VALUE;
        long translVideoMin = Long.MAX_VALUE;
        for(int i=1;i<numFrames;i++) {
            Frame thisFrame = allFrames.get(i);

            long distsum = 0;
            int numKeypoints = thisFrame.previousKeypoints.size();
            for(int k=0;k<numKeypoints;k++) {
                Point prevpt = thisFrame.previousKeypoints.get(k).pt;
                Point thispt = thisFrame.currentKeypoints.get(k).pt;

                double distsq = (thispt.x - prevpt.x)*(thispt.x - prevpt.x) + (thispt.y - prevpt.y)*(thispt.y - prevpt.y);
                distsum += Math.sqrt(distsq);
            }

            long value = distsum / numKeypoints;
            translVideo.add(value);

            if(value > translVideoMax)
                translVideoMax = value;

            if(value<translVideoMin)
                translVideoMin = value;
        }

        // Draw the per-gyro-sample translation estimate
        int numSamples = sortedTimestamps.size();
        long translGyroMin = Long.MAX_VALUE;
        long translGyroMax = Long.MIN_VALUE;
        LinkedList<Long> translGyro = new LinkedList<Long>();
        for(int i=1;i<numSamples;i++) {
            long timestamp = sortedTimestamps.get(i);
            Point3 thisSample = findClosestThetaSample(timestamp + (long) estimateParams.td * 1000000000);

            // Use the W matrix to transpose gyro samples into pixels
            Mat W = getW(thisSample, mFrameWidth, mFrameHeight, estimateParams.f);

             // TODO use the homography matrix to convert this from meters to pixels
            long value = (long)(estimateParams.f*thisSample.y);
            // long value = (long)(thisSample.x*thisSample.x + thisSample.y*thisSample.y + thisSample.z*thisSample.z);
            translGyro.add(value);

            if(value>translGyroMax)
                translGyroMax = value;

            if(value<translGyroMin)
                translGyroMin = value;
        }

        // Increase the amounts by a little bit so there's space
        long distanceVideo = translVideoMax - translVideoMin;
        long distanceGyro = translGyroMax - translGyroMin;
        translVideoMax += distanceVideo/3;
        translVideoMin -= distanceVideo/3;
        translGyroMax += distanceGyro/3;
        translGyroMin -= distanceGyro/3;

        long globalMin = Math.min(translGyroMin, translVideoMin);
        long globalMax = Math.max(translGyroMax, translVideoMax);

        // Now that we have the min/max and the ranges - we can start drawing
        Point prevPt = null;
        for(int i=0;i<numFrames-1;i++){
            int x = (int)(i*mFrameWidth/(double)numFrames);

            long value = translVideo.get(i);
            int y = (int)(mFrameHeight*(1-(value-globalMin)/(double)(globalMax-globalMin)));

            Point pt = new Point(x, y);

            Core.line(ret, pt, pt, new Scalar(255, 0, 0, 128), 3);

            if(prevPt!=null){
                Core.line(ret, pt, prevPt, new Scalar(255, 0, 0, 128), 1);
            }

            prevPt = pt;
        }

        // Clean it up for the next run
        prevPt = null;

        // Now draw the frames
        int num = translGyro.size();
        for(int i=0;i<num;i++) {
            int x = (int)(i*mFrameWidth/numSamples);

            long value = translGyro.get(i);
            int y = (int)(mFrameHeight*(1-(value-globalMin)/(double)(globalMax-globalMin)));

            Point pt = new Point(x, y);
            Core.line(ret, pt, pt, new Scalar(0, 0, 255, 128), 3);

            if(prevPt!=null){
                Core.line(ret, pt, prevPt, new Scalar(0, 0, 255, 128), 1);
            }
            // Log.d(TAG, "Drawing point " + pt.x + ", " + pt.y);
            prevPt = pt;
        }

        return ret;
    }

    /**
     * Populates the theta field in the frames here
     * @param frames
     */
    protected void integrateAngularVelocity(List<Frame> frames) {
        int numFrames = frames.size();
        Frame[] frameArray = frames.toArray(new Frame[numFrames]);
        double[] dt = new double[numFrames-1];

        // Create an array of time differences
        for(int i=1;i<numFrames;i++) {
            dt[i-1] = (frameArray[i].timestamp - frameArray[i-1].timestamp)/(double)(1000000000.0);
        }

        frames.get(0).setTheta(new Point3(0,0,0));
        Point3 sum = new Point3(0, 0, 0);

        for(int i=1;i<numFrames;i++) {
            Point3 currentOmega  = frameArray[i].getAngularVelocity();
            Point3 previousOmega = frameArray[i-1].getAngularVelocity();
            sum.x += (currentOmega.x + previousOmega.x) / 2 * dt[i-1];
            sum.y += (currentOmega.y + previousOmega.y) / 2 * dt[i-1];
            sum.z += (currentOmega.z + previousOmega.z) / 2 * dt[i-1];

            // The underlying array and List point to the same objects - so modifying one always
            // changes the other
            frameArray[i].setTheta(sum.clone());
        }

        //Log.d(TAG, "INTEGRATE Original stuff = " + frames.get(2).getAngle());
        //Log.d(TAG, "INTEGRATE New stuff = " + frameArray[2].getAngle());
    }

    protected void calculateEstimate() {
        boolean estimateFocalLength = false;

        // Some random units - approximately set to 45 degrees in the horizontal
        estimateParams.f = mFrameWidth / (2 * Math.tan(Math.PI/8));

        // seconds - The sensor delay
        estimateParams.td = 0;

        // seconds - The rolling shutter duration
        estimateParams.ts = 0;

        // These are never changed
        final double[] terminate_precision = new double[3];

        terminate_precision[0] = 0.001;

        // 100ns
        terminate_precision[1] = 0.0000001;

        // 10 microseconds
        terminate_precision[2] = 0.00001;

        double[] delta = new double[3];
        delta[0] = 0.25;
        delta[1] = 0.010;
        delta[2] = 0.010;       // 10 miliseconds

        // Error with no estimates
        double error = objective_function(allFrames, 1, estimateParams, mFrameWidth, mFrameHeight);

        boolean all_good = false;
        long iteration = 0;
        long numFrames = allFrames.size();
        long frameIndex = 1;
        double newError = error;
        while(!all_good && iteration < 100) {
            Log.d(TAG, "===============================");
            Log.d(TAG, "Working on iteration #" + iteration);

            // work on f
            estimateParams.f = estimateParams.f + delta[0];
            newError = objective_function(allFrames, frameIndex, estimateParams, mFrameWidth, mFrameHeight);
            if (newError > error) {
                estimateParams.f = estimateParams.f - delta[0];
                delta[0] = -delta[0] / 3;
            } else
                error = newError;

            // work on td
            estimateParams.td = estimateParams.td + delta[1];
            newError = objective_function(allFrames, frameIndex, estimateParams, mFrameWidth, mFrameHeight);
            if(newError > error) {
                estimateParams.td  = estimateParams.td - delta[1];
                delta[1] = -delta[1]/3;
            } else
                error = newError;

            // work on ts
            estimateParams.ts = estimateParams.ts + delta[2];
            newError = objective_function(allFrames, frameIndex, estimateParams, mFrameWidth, mFrameHeight);
            if(newError > error) {
                estimateParams.ts = estimateParams.ts - delta[2];
                delta[2] = -delta[2]/3;
            } else
                error = newError;


            Log.d(TAG, "ITERATION Focal length = " + estimateParams.f);
            Log.d(TAG, "ITERATION ts = " + estimateParams.ts);
            Log.d(TAG, "ITERATION td = " + estimateParams.td);
            Log.d(TAG, "ITERATION error = " + error);
            //Log.i(TAG, "ITERATION d = " + estimateParams.d.x + ", " + estimateParams.d.y + ", " + estimateParams.d.z);

            frameIndex++;
            if(frameIndex>=numFrames) {
                frameIndex = 1;
                // We're ignoring 1 so that 'prevFrame' can be evaluated in objective_function
            }

            all_good = true;
            for(int i=0;i<3;i++){
                if(Math.abs(delta[i]) >= terminate_precision[i]) {
                    all_good = false;
                    break;
                }
            }

            iteration++;
        }

        Log.i(TAG, "Just finished calculating the objective function. Error = " + error);
        Log.i(TAG, "ITERATION Focal length = " + estimateParams.f);
        Log.i(TAG, "ITERATION ts = " + estimateParams.ts);
        Log.i(TAG, "ITERATION td = " + estimateParams.td);
        Log.i(TAG, "ITERATION error = " + error);
        //Log.i(TAG, "ITERATION d = " + estimateParams.d.x + ", " + estimateParams.d.y + ", " + estimateParams.d.z);
    }

    /**
     * Calculates the total error
     * @param frames A list of all frames with their keypoints. The keypoints are used to calculate
     *               the error magnitude
     * @param params The current estimate of parameters
     * @return the error magnitude
     */
    protected double objective_function(List<Frame> frames, long frame_index, EstimatedCameraParameters params, long frameWidth, long frameHeight) {
        int numFrames = frames.size();
        double err = 0;

        //for(int i=1;i<numFrames;i++) {
        Frame currentFrame = frames.get((int)frame_index);
        Frame previousFrame = frames.get((int)frame_index-1);

        if(currentFrame.currentKeypoints.size() != currentFrame.previousKeypoints.size()) {
            Log.e(TAG, "The number of keypoints don't match - something is wrong");
            return Double.MAX_VALUE;
        }

        int numKeypoints = currentFrame.currentKeypoints.size();

        for(int j=0;j<numKeypoints;j++) {
            final Point ptCurrent = currentFrame.currentKeypoints.get(j).pt;
            final Point ptPrevious = currentFrame.previousKeypoints.get(j).pt;

            final double w1 = (ptPrevious.y - frameHeight/2) / (double)frameHeight;
            final double t1 = previousFrame.timestamp/1000000.0 + params.td + params.ts * w1;
            final Point3 th1 = findClosestThetaSample((long) (t1 * 1000000000));
            final Point3 dr1 = findClosestDriftSample((long)(t1*1000000000));

            final double w2 = (ptCurrent.y - frameHeight/2) / (double)frameHeight;
            final double t2 = currentFrame.timestamp/1000000.0 + params.td + params.ts * w2;
            final Point3 th2 = findClosestThetaSample((long) (t2 * 1000000000));
            final Point3 dr2 = findClosestDriftSample((long)(t2*1000000000));

            final double dt = t2 - t1;
            final Point3 dth = new Point3(th2.x - th1.x,
                                          th2.y - th1.y,
                                          th2.z - th1.z);

            Mat prePX = getW(dth, frameWidth, frameHeight, params.f);

            double[] a = new double[3];
            a[0] = ptPrevious.x; a[1] = ptPrevious.y; a[2] = 1;
            Mat prevPtMat = new Mat(3, 1, CvType.CV_64FC1);
            prevPtMat.put(0, 0, a);

            Mat pXMat = new Mat(3, 1, CvType.CV_64FC1);

            // General matrix multiply
            Core.gemm(prePX, prevPtMat, 1, new Mat(), 0, pXMat);

            pXMat.get(0, 0, a);
            Point3 pX = new Point3(a[0], a[1], a[2]);

            // cv::Point2f dxy = frame_next.pt[j] - (cv::Point2f(pX.x, pX.y) / pX.z);
            Point dxy = new Point();
            dxy.x = ptCurrent.x - pX.x / pX.z;
            dxy.y = ptCurrent.y - pX.y / pX.z;

            //Log.d(TAG, "Tried matching ("+ptCurrent.x+","+ptCurrent.y+") and ("+ptPrevious.x+","+ptPrevious.y+")");
            //Log.d(TAG, "After transform found:("+ptCurrent.x+","+ptCurrent.y+") and ("+(pX.x/pX.z)+","+(pX.y/pX.z)+")");

            err += dxy.dot(dxy);
        }

        return err / numKeypoints;
    }

    protected Mat getW(Point3 theta, double width, double height, double f) {
        Mat K = Mat.eye(3, 3, CvType.CV_64FC1);

        double[] arrayK = new double[9];
        arrayK[0] = 1; arrayK[1] = 0; arrayK[2] = -width/2*f;
        arrayK[3] = 0; arrayK[4] = 1; arrayK[5] = -height/2*f;
        arrayK[6] = 0; arrayK[7] = 0; arrayK[8] = 1/f;
        K.put(0, 0, arrayK);

        /*double[] arrayInvK = new double[9];
        arrayInvK[0] = 1; arrayInvK[1] = 0; arrayInvK[2] = -width/(2*f);
        arrayInvK[3] = 0; arrayInvK[4] = 1; arrayInvK[5] = -height/(2*f);
        arrayInvK[6] = 0; arrayInvK[7] = 0; arrayInvK[8] = 1/f;
        invK.put(0, 0, arrayInvK);*/

        Mat invK = null;
        invK = K.inv();
        Mat prod1 = Mat.zeros(3, 3, CvType.CV_64FC1);
        Mat prod2 = Mat.zeros(3, 3, CvType.CV_64FC1);
        Mat prod3 = Mat.zeros(3, 3, CvType.CV_64FC1);
        Mat ret = Mat.zeros(3, 3, CvType.CV_64FC1);



        if(permutation == 0) {
            // error = 6.50662604040778E12
            Core.gemm(roty_matrix(theta.y), K, 1, new Mat(), 0, prod1);
            Core.gemm(rotx_matrix(theta.x), prod1, 1, new Mat(), 0, prod2);
            Core.gemm(rotz_matrix(-theta.z), prod2, 1, new Mat(), 0, prod3);
            Core.gemm(invK, prod3, 1, new Mat(), 0, ret);
        } else if(permutation==1) {
            Core.gemm(roty_matrix(theta.y), K, 1, new Mat(), 0, prod1);
            Core.gemm(rotz_matrix(-theta.z), prod1, 1, new Mat(), 0, prod2);
            Core.gemm(rotx_matrix(theta.x), prod2, 1, new Mat(), 0, prod3);
            Core.gemm(invK, prod3, 1, new Mat(), 0, ret);
        } else if(permutation==2) {
            Core.gemm(rotx_matrix(theta.x), K, 1, new Mat(), 0, prod1);
            Core.gemm(rotz_matrix(-theta.z), prod1, 1, new Mat(), 0, prod2);
            Core.gemm(roty_matrix(theta.y), prod2, 1, new Mat(), 0, prod3);
            Core.gemm(invK, prod3, 1, new Mat(), 0, ret);
        } else if(permutation==3) {
            Core.gemm(rotx_matrix(theta.x), K, 1, new Mat(), 0, prod1);
            Core.gemm(roty_matrix(theta.y), prod1, 1, new Mat(), 0, prod2);
            Core.gemm(rotz_matrix(-theta.z), prod2, 1, new Mat(), 0, prod3);
            Core.gemm(invK, prod3, 1, new Mat(), 0, ret);
        } else if(permutation==4) {
            Core.gemm(rotz_matrix(-theta.z), K, 1, new Mat(), 0, prod1);
            Core.gemm(roty_matrix(theta.y), prod1, 1, new Mat(), 0, prod2);
            Core.gemm(rotx_matrix(theta.x), prod2, 1, new Mat(), 0, prod3);
            Core.gemm(invK, prod3, 1, new Mat(), 0, ret);
        } else if(permutation==5) {
            Core.gemm(rotz_matrix(-theta.z), K, 1, new Mat(), 0, prod1);
            Core.gemm(rotx_matrix(theta.x), prod1, 1, new Mat(), 0, prod2);
            Core.gemm(roty_matrix(theta.y), prod2, 1, new Mat(), 0, prod3);
            Core.gemm(invK, prod3, 1, new Mat(), 0, ret);
        }
        return ret;
    }

    protected Mat rotx_matrix(double angle) {
        Mat ret = new Mat(3, 3, CvType.CV_64FC1);
        double[] array = new double[9];

        array[0] = 1; array[1] = 0; array[2] = 0;
        array[3] = 0; array[4] = Math.cos(angle); array[5] = -Math.sin(angle);
        array[6] = 0; array[7] = Math.sin(angle); array[8] = Math.cos(angle);

        ret.put(0, 0, array);

        return ret;
    }

    protected Mat roty_matrix(double angle) {
        Mat ret = new Mat(3, 3, CvType.CV_64FC1);
        double[] array = new double[9];

        array[0] = Math.cos(angle); array[1] = 0; array[2] = Math.sin(angle);
        array[3] = 0; array[4] = 1; array[5] = 0;
        array[6] = -Math.sin(angle); array[7] = 0; array[8] = Math.cos(angle);

        ret.put(0, 0, array);

        return ret;
    }

    protected Mat rotz_matrix(double angle) {
        Mat ret = new Mat(3, 3, CvType.CV_64FC1);
        double[] array = new double[9];

        array[0] = Math.cos(angle); array[1] = -Math.sin(angle); array[2] = 0;
        array[3] = Math.sin(angle); array[4] = Math.cos(angle); array[5] = 0;
        array[6] = 0; array[7] = 0; array[8] = 1;

        ret.put(0, 0, array);

        return ret;
    }

    protected Mat convertToBgr(Mat image) {
        Mat ret = new Mat(image.rows(), image.cols(), CvType.CV_8UC3);
        Imgproc.cvtColor(image, ret, Imgproc.COLOR_RGBA2BGR);
        return ret;
    }

    @Override
    public void onFrameAvailable(Frame frame) {
        Log.d(TAG, "Received frame as a Mat");

        Mat frameBgr = convertToBgr(frame.getImage());

        // Generate the preview frame
        previewFrame = frame.clone();
        previewFrame.deleteImage();

        previewMat = frameBgr.clone();

        // The 1000 is to convert microseconds into nanoseconds
        Point3 gyroSample = findClosestGyroSample(frame.timestamp * 1000);
        Point3 driftSample = findClosestDriftSample(frame.timestamp * 1000);
        Point3 thetaSample = findClosestThetaSample(frame.timestamp * 1000);

        if(gyroSample != null) {
            previewFrame.setAngularVelocity(gyroSample);
            // Log.d(TAG, "Timestamp = " + frame.timestamp + " with gyro = " + gyroSample.x + ","+gyroSample.y + "," + gyroSample.z);
        } else {
            Log.e(TAG, "gyro was null");
        }

        if(thetaSample != null) {
            previewFrame.setTheta(thetaSample);
        } else {
            Log.e(TAG, "theta was null");
        }

        if(driftSample != null) {
            previewFrame.setDrift(driftSample);
        } else {
            Log.e(TAG, "drift was null");
        }

        // Generate keypoints
        MatOfKeyPoint keypoints = new MatOfKeyPoint();
        FeatureDetector f = FeatureDetector.create(FeatureDetector.BRISK);
        f.detect(frameBgr, keypoints);

        DescriptorExtractor de = DescriptorExtractor.create(DescriptorExtractor.OPPONENT_FREAK);
        Mat currentDescriptors = new Mat();
        de.compute(frameBgr, keypoints, currentDescriptors);

        Log.d(TAG, "KEYPOINT generation complete");

        // Try and compare to previous
        if(previousFrame!=null) {
            DescriptorMatcher dm = DescriptorMatcher.create(DescriptorMatcher.BRUTEFORCE_HAMMING);
            MatOfDMatch matches = new MatOfDMatch();
            dm.match(currentDescriptors, previousKeypointDescriptors, matches);
            KeyPoint[] newkeypts = keypoints.toArray();

            List<Point> prev_points = new LinkedList<Point>();
            List<Point> this_points= new LinkedList<Point>();
            List<DMatch> good_matches = new LinkedList<DMatch>();
            for(DMatch m : matches.toList()) {
                KeyPoint prevpt = previousKeypoints[m.trainIdx];
                KeyPoint thispt = newkeypts[m.queryIdx];

                prev_points.add(prevpt.pt);
                this_points.add(thispt.pt);
            }

            Mat mask = new Mat();
            MatOfPoint2f matPrev = new MatOfPoint2f();
            MatOfPoint2f matThis = new MatOfPoint2f();

            matPrev.fromList(prev_points);
            matThis.fromList(this_points);

            // Find the homography and also the inliers
            Mat homography = Calib3d.findHomography(matPrev, matThis, Calib3d.RANSAC, 3, mask);
            previewFrame.setHomography(homography);

            int matchers = 0;
            DMatch[] listMatches = matches.toArray();
            int numMatches = mask.rows();
            for(int i=0;i<numMatches;i++) {
                if(mask.get(i, 0)[0] == 0)
                    continue;

                good_matches.add(listMatches[i]);
                matchers++;
            }

            LinkedList<KeyPoint> ptCurrent = new LinkedList<KeyPoint>();
            LinkedList<KeyPoint> ptPrevious = new LinkedList<KeyPoint>();
            for(DMatch m : good_matches) {
                KeyPoint prevpt = previousKeypoints[m.trainIdx];
                KeyPoint thispt = newkeypts[m.queryIdx];

                Core.line(previewMat, prevpt.pt, thispt.pt, new Scalar(0, 255, 0), 2);

                ptCurrent.add(thispt);
                ptPrevious.add(prevpt);
            }

            // We found matches!
            previewFrame.setMatchingKeypoints(ptCurrent, ptPrevious);

            // At this point, good_matches contains all the good points we're interested in!
            Log.d(TAG, "Matched " + matchers + " keypoints in the current and previous frame!");
        }

        allFrames.add(previewFrame);

        // Update the previous frame
        previousFrame = frame.clone();
        previousKeypoints = keypoints.toArray();
        previousKeypointDescriptors = currentDescriptors.clone();
    }

    private Point3 findClosestThetaSample(long timestamp) {
        int numTimestamps = sortedThetaTimestamps.size();
        for(int i=0;i<numTimestamps;i++) {
            long value = sortedThetaTimestamps.get(i);
            if(timestamp==value) {
                // We just overshot - find the exact value now
                return thetaSamples.get(value);
            } else if(timestamp<value) {
                long prevTimestamp = sortedThetaTimestamps.get(i-1);

                Point3 thisSample = thetaSamples.get(value);
                Point3 prevSample = thetaSamples.get(prevTimestamp);

                // Difference in timestamps in nanoseconds
                long dt = value - prevTimestamp;
                long dy = timestamp - prevTimestamp;
                long slope = dy / dt;

                Point3 ret = new Point3();
                ret.x = prevSample.x * (1-slope) + thisSample.x*slope;
                ret.y = prevSample.y * (1-slope) + thisSample.y*slope;
                ret.z = prevSample.z * (1-slope) + thisSample.z*slope;

                return ret;
            }
        }

        return null;
    }

    private Point3 findClosestGyroSample(long timestamp) {
        int numTimestamps = sortedTimestamps.size();
        for(int i=0;i<numTimestamps;i++) {
            long value = sortedTimestamps.get(i);
            if(timestamp==value) {
                // We just overshot - find the exact value now
                return gyroSamples.get(value);
            } else if(timestamp<value) {
                long prevTimestamp = sortedTimestamps.get(i-1);

                Point3 thisSample = gyroSamples.get(value);
                Point3 prevSample = gyroSamples.get(prevTimestamp);

                // Difference in timestamps in nanoseconds
                long dt = value - prevTimestamp;
                long dy = timestamp - prevTimestamp;
                long slope = dy / dt;

                Point3 ret = new Point3();
                if(prevSample == null || thisSample == null) {
                    Log.d(TAG, "Something's up");
                }
                ret.x = prevSample.x * (1-slope) + thisSample.x*slope;
                ret.y = prevSample.y * (1-slope) + thisSample.y*slope;
                ret.z = prevSample.z * (1-slope) + thisSample.z*slope;

                return ret;
            }
        }

        return null;
    }

    private Point3 findClosestDriftSample(long timestamp) {
        int numTimestamps = sortedTimestamps.size();
        for(int i=0;i<numTimestamps;i++) {
            long value = sortedTimestamps.get(i);
            if(timestamp==value) {
                // We just overshot - find the exact value now
                return driftSamples.get(value);
            } else if(timestamp<value) {
                long prevTimestamp = sortedTimestamps.get(i-1);

                Point3 thisSample = driftSamples.get(value);
                Point3 prevSample = driftSamples.get(prevTimestamp);

                // Difference in timestamps in nanoseconds
                long dt = value - prevTimestamp;
                long dy = timestamp - prevTimestamp;
                long slope = dy / dt;

                try {
                    Point3 ret = new Point3();
                    ret.x = prevSample.x * (1 - slope) + thisSample.x * slope;
                    ret.y = prevSample.y * (1 - slope) + thisSample.y * slope;
                    ret.z = prevSample.z * (1 - slope) + thisSample.z * slope;
                    return ret;
                } catch(Exception e) {
                    e.printStackTrace();
                    i--;
                }
            }
        }

        return null;
    }

    private Mat findClosestHomography(List<Frame> frames, long timestamp) {
        Mat ret = null;

        int numFrames = frames.size();
        for(int i=0;i<numFrames;i++) {
            Frame frame = frames.get(i);

            // Frame timestamps are in microseconds - timestamp is in nanoseconds
            if(frame.timestamp*1000 > timestamp) {
                return frame.homography;
            }
        }

        return ret;
    }

    @Override
    public void onFrameComplete(long frameDone) {
        final long d = frameDone;
        // Inform the main thread to update the progress bar
        if(mProgressListener!=null) {

            // Should run
            mMainHandler.post(new Runnable() {
                @Override
                public void run() {
                    if(mCameraView!=null)
                        mCameraView.setPreviewFrame(previewMat);

                    mProgressListener.onProgress(d);
                }
            });
        }
    }

    public void setGyroSamples(Map<Long, Point3> gyroSamples, Map<Long, Point3> driftSamples) {
        this.gyroSamples = gyroSamples;
        this.driftSamples = driftSamples;
        sortedTimestamps = new LinkedList<Long>(gyroSamples.keySet());
        Collections.sort(sortedTimestamps);
    }

    public void setThetaSamples(Map<Long, Point3> thetaSamples) {
        this.thetaSamples = thetaSamples;
        sortedThetaTimestamps = new LinkedList<Long>(thetaSamples.keySet());
        Collections.sort(sortedThetaTimestamps);
    }

    public interface CalibrateWorkerProgressListener {
        public boolean onProgress(long frameNumber);
        public boolean onFinish(EstimatedCameraParameters params, Mat alignmentGraph);
    }
}


// All four parameters that this class returns
class EstimatedCameraParameters {
    public double f;       // The focus length
    public double td;      // The difference between frame and gyro timestamps
    public double ts;      // Rolling shutter duration
}