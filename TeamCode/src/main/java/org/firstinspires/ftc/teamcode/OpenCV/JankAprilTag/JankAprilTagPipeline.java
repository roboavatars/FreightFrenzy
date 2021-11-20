package org.firstinspires.ftc.teamcode.OpenCV.JankAprilTag;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

@Config
public class JankAprilTagPipeline extends OpenCvPipeline {

    // Cases
    public enum Case {None, Left, Middle, Right}

    // Image Cropping
    public static int RECT_X = 0;
    public static int RECT_Y = 0;
    public static int RECT_WIDTH = 640;
    public static int RECT_HEIGHT = 320;
    public static int RETURN_IMAGE = 1;

    // Debug
    public static boolean debug = false;

    // Results
    private Case outputCase = Case.None;
    private Case[] results = new Case[] {Case.None, Case.None, Case.None, Case.None, Case.None};
    private int cycles = 0;

    // CV Thresholds
    public static int MIN_H = 40;
    public static int MIN_S = 35;
    public static int MIN_V = 10;
    public static int MAX_H = 80;
    public static int MAX_S = 200;
    public static int MAX_V = 255;
    public static int AREA_MIN = 7000;

    // Image Processing Mats
    private Mat hsv = new Mat();
    private Mat processed = new Mat();

    public JankAprilTagPipeline() {}

    @Override
    public Mat processFrame(Mat input) {

        // Crop Input Image
        input = new Mat(input, new Rect(RECT_X, RECT_Y, RECT_WIDTH, RECT_HEIGHT));

        // Draw Three Red Rectangles
        if (debug) {
            Imgproc.rectangle(input, new Point(0, 0), new Point(RECT_WIDTH/3, RECT_HEIGHT), new Scalar(255, 0, 0), 4);
            Imgproc.rectangle(input, new Point(RECT_WIDTH/3, 0), new Point(2*RECT_WIDTH/3, RECT_HEIGHT), new Scalar(255, 0, 0), 4);
            Imgproc.rectangle(input, new Point(2*RECT_WIDTH/3, 0), new Point(RECT_WIDTH, RECT_HEIGHT), new Scalar(255, 0, 0), 4);
        }

        // Convert to HSV Color Space
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
        Core.inRange(hsv, new Scalar(MIN_H, MIN_S, MIN_V), new Scalar(MAX_H, MAX_S, MAX_V), processed);

        // Remove Noise
        Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_OPEN, new Mat());
        Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE, new Mat());

        // Split Into Three Regions
        Mat left = new Mat(processed, new Rect(0, 0, RECT_WIDTH/3, RECT_HEIGHT));
        Mat middle = new Mat(processed, new Rect(RECT_WIDTH/3, 0, RECT_WIDTH/3, RECT_HEIGHT));
        Mat right = new Mat(processed, new Rect(2*RECT_WIDTH/3, 0, RECT_WIDTH/3, RECT_HEIGHT));

        // Compute White Area for each Region
        int leftArea = Core.countNonZero(left);
        int middleArea = Core.countNonZero(middle);
        int rightArea = Core.countNonZero(right);

        // Find Area with Maximum Area
        int maxArea = leftArea;
        outputCase = Case.Left;

        if (middleArea > maxArea) {
            maxArea = middleArea;
            outputCase = Case.Middle;
        }

        if (rightArea > maxArea) {
            maxArea = rightArea;
            outputCase = Case.Right;
        }

        // Reject Area if too Small
        if (maxArea < AREA_MIN) {
            outputCase = Case.None;
        }

        // Draw Green Rectangle Depending on Region
        if (debug) {
            if (outputCase == Case.Left) {
                Imgproc.rectangle(input, new Point(0, 0), new Point(RECT_WIDTH/3, RECT_HEIGHT), new Scalar(0, 255, 0), 4);
            } else if (outputCase == Case.Middle) {
                Imgproc.rectangle(input, new Point(RECT_WIDTH/3, 0), new Point(2*RECT_WIDTH/3, RECT_HEIGHT), new Scalar(0, 255, 0), 4);
            } else if (outputCase == Case.Right) {
                Imgproc.rectangle(input, new Point(2*RECT_WIDTH/3, 0), new Point(RECT_WIDTH, RECT_HEIGHT), new Scalar(0, 255, 0), 4);
            }
        }

        log("Case: " + outputCase.name());

        results[cycles % 5] = outputCase;
        cycles++;

        if (RETURN_IMAGE == 0) {
            return processed;
        } else {
            return input;
        }
    }

    public Case getResult() {
        List<Case> list = Arrays.asList(results);

        int none = Collections.frequency(list, Case.None);
        int left = Collections.frequency(list, Case.Left);
        int middle = Collections.frequency(list, Case.Middle);
        int right = Collections.frequency(list, Case.Right);

        if (none > left && none > middle && none > right) {
            return Case.None;
        } else if (left > none && left > middle && left > right) {
            return Case.Left;
        } else if (middle > none && middle > left && middle > right) {
            return Case.Middle;
        } else {
            return Case.Right;
        }
    }

    private void log(String message) {
        Log.w("jank-april-tag-pipe", message);
    }
}