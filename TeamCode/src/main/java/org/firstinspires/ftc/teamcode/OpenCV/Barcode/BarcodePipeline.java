package org.firstinspires.ftc.teamcode.OpenCV.Barcode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

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
    public static int RECT_WIDTH = 280;
    public static int RECT_HEIGHT = 40;
    public static int BLUE_LEFT_DIVIDER = 80;
    public static int BLUE_RIGHT_DIVIDER = 180;
    public static int RED_LEFT_DIVIDER = 100;
    public static int RED_RIGHT_DIVIDER = 200;
    public static int RETURN_IMAGE = 0;

    public static int x = 20;
    public static int y = 130;

    public int leftDivider;
    public int rightDivider;

    // CV Thresholds
    public static int BLUE_MIN_H = 140;
    public static int BLUE_MIN_S = 100;
    public static int BLUE_MIN_V = 70;
    public static int BLUE_MAX_H = 190;
    public static int BLUE_MAX_S = 255;
    public static int BLUE_MAX_V = 255;
    public static int RED_MIN_H = 120;
    public static int RED_MIN_S = 40;
    public static int RED_MIN_V = 90;
    public static int RED_MAX_H = 140;
    public static int RED_MAX_S = 255;
    public static int RED_MAX_V = 255;
//    public static int RED_MIN_H_2 = 225;
//    public static int RED_MAX_H_2 = 255;

    public static int AREA_MAX = 200;
    public static int SQUARE_AREA = 600;

    public int leftArea = -1;
    public int middleArea = -1;

    // Image Processing Mats
    private Mat hsv = new Mat();
    private Mat processed = new Mat();
    private Mat save;
    private Mat left = new Mat();
    private Mat middle = new Mat();
    private Mat blank = new Mat();

    // Debug
    private static String path = "/sdcard/OpenCV/barcode/";
    public boolean debug = true;
    public boolean isRed;

    // Results
    private Case outputCase = Case.None;
    private Case[] results = new Case[] {Case.None, Case.None, Case.None, Case.None, Case.None};
    private int cycles = 0;

    public BarcodePipeline(boolean isRed) {
        this.isRed = isRed;

        if (isRed) {
            leftDivider = RED_LEFT_DIVIDER;
            rightDivider = RED_RIGHT_DIVIDER;
        } else {
            leftDivider = BLUE_LEFT_DIVIDER;
            rightDivider = BLUE_RIGHT_DIVIDER;
        }
    }

    @Override
    public Mat processFrame(Mat input) {
//         Crop Input Image
        input = new Mat(input, new Rect(x, y, RECT_WIDTH, RECT_HEIGHT));
        saveMatToDisk("input", input);

        // Draw Three Red Rectangles
        if (debug) {
            Imgproc.rectangle(input, new Point(0, 0), new Point(leftDivider, RECT_HEIGHT), new Scalar(255, 0, 0), 4);
            Imgproc.rectangle(input, new Point(leftDivider, 0), new Point(rightDivider, RECT_HEIGHT), new Scalar(255, 0, 0), 4);
            Imgproc.rectangle(input, new Point(rightDivider, 0), new Point(RECT_WIDTH, RECT_HEIGHT), new Scalar(255, 0, 0), 4);
        }

        // Convert to HSV Color Space
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
        if (isRed) {
            Core.inRange(hsv, new Scalar(RED_MIN_H, RED_MIN_S, RED_MIN_V), new Scalar(RED_MAX_H, RED_MAX_S, RED_MAX_V), processed);
        } else {
            Core.inRange(hsv, new Scalar(BLUE_MIN_H, BLUE_MIN_S, BLUE_MIN_V), new Scalar(BLUE_MAX_H, BLUE_MAX_S, BLUE_MAX_V), processed);
        }

        // Remove Noise
        Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_OPEN, blank);
        Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE, blank);
        saveMatToDisk("processed", processed);

        // Split Into Three Regions
        left = new Mat(processed, new Rect(0, 0, leftDivider, RECT_HEIGHT));
        middle = new Mat(processed, new Rect(rightDivider, 0, RECT_WIDTH - rightDivider, RECT_HEIGHT));

        // Compute White Area for each Region
        leftArea = Core.countNonZero(left);
        middleArea = Core.countNonZero(middle);
//        try {
            addPacket("leftSize", leftArea);
            addPacket("middleSize", middleArea);
//        } catch (Exception e) {
//            addPacket("error", e);
//        }

        sendPacket();

        // Find Area with Minimum Area
        int minArea = leftArea;
        outputCase = Case.Left;

        if (middleArea < minArea) {
            minArea = middleArea;
            outputCase = Case.Middle;
        }

        if (leftArea > SQUARE_AREA && middleArea > SQUARE_AREA) {
            outputCase = Case.Right;
        }
        // Reject Area if Too Large
        else if (minArea > AREA_MAX) {
            outputCase = Case.None;
        }

        // Draw Green Rectangle Depending on Region
        if (debug) {
            if (outputCase == Case.Left) {
                Imgproc.rectangle(input, new Point(0, 0), new Point(leftDivider, RECT_HEIGHT), new Scalar(0, 255, 0), 4);
            } else if (outputCase == Case.Middle) {
                Imgproc.rectangle(input, new Point(rightDivider, 0), new Point(RECT_WIDTH, RECT_HEIGHT), new Scalar(0, 255, 0), 4);
            }
        }
        saveMatToDisk("greenRect", input);

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
        addPacket("left", leftArea);
        addPacket("right", middleArea);

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
        Imgcodecs.imwrite(path + name + ".jpg", save);
    }
}
