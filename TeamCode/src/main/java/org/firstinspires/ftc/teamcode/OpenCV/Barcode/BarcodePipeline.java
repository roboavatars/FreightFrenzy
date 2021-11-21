package org.firstinspires.ftc.teamcode.OpenCV.Barcode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

@Config
public class BarcodePipeline extends OpenCvPipeline {

    // Cases
    public enum Case {None, Left, Middle, Right}

    // Image Cropping
    public static int RECT_X = 0;
    public static int RECT_Y = 0;
    public static int RECT_WIDTH = 640;
    public static int RECT_HEIGHT = 320;
    public static int BLUE_LEFT_DIVIDER = 200;
    public static int BLUE_RIGHT_DIVIDER = 400;
    public static int RED_LEFT_DIVIDER = 400;
    public static int RED_RIGHT_DIVIDER = 640;
    public static int RETURN_IMAGE = 1;
    public static int leftDivider;
    public static int rightDivider;

    // Debug
    public static boolean debug = true;

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
    public static int AREA_MIN = 5000;

    // Image Processing Mats
    private Mat hsv = new Mat();
    private Mat processed = new Mat();
    private Mat save;

    private String path = "/sdcard/EasyOpenCV/";

    public BarcodePipeline(boolean isRed) {
        if (isRed){
            leftDivider = RED_LEFT_DIVIDER;
            rightDivider = RED_RIGHT_DIVIDER;
        } else {
            leftDivider = BLUE_LEFT_DIVIDER;
            rightDivider = BLUE_RIGHT_DIVIDER;
        }
    }

    @Override
    public Mat processFrame(Mat input) {

        // Crop Input Image
        input = new Mat(input, new Rect(RECT_X, RECT_Y, RECT_WIDTH, RECT_HEIGHT));
        saveMatToDisk("input.jpg", input);

        // Draw Three Red Rectangles
        if (debug) {
            Imgproc.rectangle(input, new Point(0, 0), new Point(leftDivider, RECT_HEIGHT), new Scalar(255, 0, 0), 4);
            Imgproc.rectangle(input, new Point(leftDivider, 0), new Point(rightDivider, RECT_HEIGHT), new Scalar(255, 0, 0), 4);
            Imgproc.rectangle(input, new Point(rightDivider, 0), new Point(RECT_WIDTH, RECT_HEIGHT), new Scalar(255, 0, 0), 4);
//            saveMatToDisk("redRect.jpg", input);
        }

        // Convert to HSV Color Space
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
//        saveMatToDisk("hsv.jpg", hsv);
        Core.inRange(hsv, new Scalar(MIN_H, MIN_S, MIN_V), new Scalar(MAX_H, MAX_S, MAX_V), processed);
//        saveMatToDisk("processed.jpg", processed);

        // Remove Noise
        Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_OPEN, new Mat());
        Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE, new Mat());
        saveMatToDisk("processed2.jpg", processed);

        // Split Into Three Regions
        Mat left = new Mat(processed, new Rect(0, 0, leftDivider, RECT_HEIGHT));
        Mat middle = new Mat(processed, new Rect(leftDivider, 0, rightDivider - leftDivider, RECT_HEIGHT));
        Mat right = new Mat(processed, new Rect(rightDivider, 0, RECT_WIDTH - rightDivider, RECT_HEIGHT));
//        saveMatToDisk("left.jpg", left);
//        saveMatToDisk("middle.jpg", middle);
//        saveMatToDisk("right.jpg", right);

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
        saveMatToDisk("greenRect.jpg", input);

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

        if (left > none && left > middle && left > right) {
            return Case.Left;
        } else if (middle > none && middle > left && middle > right) {
            return Case.Middle;
        } else {
            return Case.Right;
        }
    }

    private void log(String message) {
        Log.w("barcode-pipe", message);
    }

    public void saveMatToDisk(String name, Mat mat) {
        save = mat.clone();
        Imgproc.cvtColor(mat, save, Imgproc.COLOR_BGR2RGB);
        Imgcodecs.imwrite(path + name, save);
    }
}
