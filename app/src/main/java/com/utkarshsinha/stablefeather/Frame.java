package com.utkarshsinha.stablefeather;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.features2d.KeyPoint;

import java.util.List;

/**
 * Created by utkarsh on 12/29/14.
 */
public class Frame implements Cloneable {
    // Fetched from sensors
    protected Mat     matImage;  // The image
    protected long    timestamp; // The timestamp of the frame
    protected Point3  omega;     // fetched from the sensor manaer
    protected Point3  drift;     // fetched from the sensor manager
    public Point3 theta;         // fetched from the sensor manager
    protected List<KeyPoint> currentKeypoints;
    protected List<KeyPoint> previousKeypoints;
    protected Mat homography;

    // Constructor
    public Frame(Mat img, long prestime, Point3 o, Point3 d) {
        matImage = img;
        timestamp = prestime;
        omega = o;
        drift = d;

        theta = new Point3(0, 0, 0);
    }

    /**
     * @return the raw image pointer for this frame
     */
    public Mat getImage() {
        return matImage;
    }

    /**
     * Fetch the raw angular velocity
     * @return
     */
    public Point3 getAngularVelocity() {
        return omega;
    }

    /**
     * Needs to be calculated somewhere
     * @return
     */
    public Point3 getAngle() {
        return theta;
    }

    public void setAngularVelocity(Point3 ang) {
        omega = ang.clone();
    }

    public void setTheta(Point3 t) {
        theta = t.clone();
    }

    public void setDrift(Point3 d) {
        drift = d.clone();
    }

    /**
     * Useful when you don't need to use the image
     */
    public void deleteImage() {
        matImage.release();
    }

    @Override
    protected Frame clone() {
        Frame newFrame = new Frame(matImage, timestamp, omega, drift);
        newFrame.theta = theta;
        return newFrame;
    }

    public void setHomography(Mat hom) {
        homography = hom.clone();
    }

    /**
     * The "current" keypoints and the corresponding "previous" keypoints should be set with
     * this method. We're only interested in matched keypoints
     * @param ptsCurrent the current keypoints
     * @param ptsPrev the old keypoints
     */
    public void setMatchingKeypoints(List<KeyPoint> ptsCurrent, List<KeyPoint> ptsPrev) {
        currentKeypoints = ptsCurrent;
        previousKeypoints = ptsPrev;
    }
}
