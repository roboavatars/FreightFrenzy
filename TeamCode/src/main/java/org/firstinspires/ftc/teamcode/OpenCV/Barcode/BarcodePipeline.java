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
    public static int RECT_WIDTH = 280;
    public static int RECT_HEIGHT = 40;
    public static int BLUE_LEFT_DIVIDER = 80;
    public static int BLUE_RIGHT_DIVIDER = 135;
    public static int RED_LEFT_DIVIDER = 120;
    public static int RED_RIGHT_DIVIDER = 180;

    // Carousel
    public static int BLUE_LEFT_DIVIDER_C = 130;
    public static int BLUE_RIGHT_DIVIDER_C = 180;
    public static int RED_LEFT_DIVIDER_C = 90;
    public static int RED_RIGHT_DIVIDER_C = 165;

    public static int RETURN_IMAGE = 0;

    public static int xw = 25;
    public static int xc = 10;
    public static int y = 130;

    public int leftDivider;
    public int rightDivider;

    // CV Thresholds
    public static int BLUE_MIN_H = 0;
    public static int BLUE_MIN_S = 40;
    public static int BLUE_MIN_V = 90;
    public static int BLUE_MAX_H = 50;
    public static int BLUE_MAX_S = 255;
    public static int BLUE_MAX_V = 255;
    public static int RED_MIN_H = 120;
    public static int RED_MIN_S = 40;
    public static int RED_MIN_V = 90;
    public static int RED_MAX_H = 140;
    public static int RED_MAX_S = 255;
    public static int RED_MAX_V = 255;

    public static int AREA_MAX = 200;
    public static int SQUARE_AREA = 90;

    public int leftArea = -1;
    public int middleArea = -1;
    public int rightArea = -1;

    // Image Processing Mats
    private Mat hsv = new Mat();
    private Mat processed = new Mat();
    private Mat save;
    private Mat left = new Mat();
    private Mat middle = new Mat();
    private Mat right = new Mat();
    private Mat blank = new Mat();

    // Debug
    private static String path = "/sdcard/EasyOpenCV/";
    public boolean debug = true;
    public boolean isRed;
    public boolean isWarehouse;

    // Results
    private Case outputCase = Case.None;
    private Case[] results = new Case[] {Case.None, Case.None, Case.None, Case.None, Case.None};
    private int cycles = 0;

    public BarcodePipeline(boolean isRed, boolean isWarehouse) {
        this.isRed = isRed;
        this.isWarehouse = isWarehouse;

        if (isRed) {
            if (isWarehouse) {
                leftDivider = RED_LEFT_DIVIDER;
                rightDivider = RED_RIGHT_DIVIDER;
            } else {
                leftDivider = RED_LEFT_DIVIDER_C;
                rightDivider = RED_RIGHT_DIVIDER_C;
                RECT_WIDTH = 250;
            }
        } else {
            if (isWarehouse) {
                leftDivider = BLUE_LEFT_DIVIDER;
                rightDivider = BLUE_RIGHT_DIVIDER;
                RECT_WIDTH = 230;
            } else {
                leftDivider = BLUE_LEFT_DIVIDER_C;
                rightDivider = BLUE_RIGHT_DIVIDER_C;
            }
        }
    }

    @Override
    public Mat processFrame(Mat input) {
//         Crop Input Image
        if (isWarehouse) {
            input = new Mat(input, new Rect(xw, y, RECT_WIDTH, RECT_HEIGHT));
        } else {
            input = new Mat(input, new Rect(xc, y, RECT_WIDTH, RECT_HEIGHT));
        }
        saveMatToDisk("input", input);

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

        if (isRed && isWarehouse || !isRed && !isWarehouse) {
            // Split Into Three Regions
            left = new Mat(processed, new Rect(0, 0, leftDivider, RECT_HEIGHT));
            middle = new Mat(processed, new Rect(rightDivider, 0, RECT_WIDTH - rightDivider, RECT_HEIGHT));

            // Compute White Area for each Region
            leftArea = Core.countNonZero(left);
            middleArea = Core.countNonZero(middle);

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
        } else {
            // Split Into Three Regions
            middle = new Mat(processed, new Rect(0, 0, leftDivider, RECT_HEIGHT));
            right = new Mat(processed, new Rect(rightDivider, 0, RECT_WIDTH - rightDivider, RECT_HEIGHT));

            // Compute White Area for each Region
            middleArea = Core.countNonZero(middle);
            rightArea = Core.countNonZero(right);

            // Find Area with Minimum Area
            int minArea = middleArea;
            outputCase = Case.Middle;

            if (rightArea < minArea) {
                minArea = rightArea;
                outputCase = Case.Right;
            }

            if (middleArea > SQUARE_AREA && rightArea > SQUARE_AREA) {
                outputCase = Case.Left;
            }
            // Reject Area if Too Large
            else if (minArea > AREA_MAX) {
                outputCase = Case.None;
            }
        }

        // Draw Three Red Rectangles
        if (debug) {
            Imgproc.rectangle(input, new Point(0, 0), new Point(leftDivider, RECT_HEIGHT), new Scalar(255, 0, 0), 4);
            Imgproc.rectangle(input, new Point(leftDivider, 0), new Point(rightDivider, RECT_HEIGHT), new Scalar(255, 0, 0), 4);
            Imgproc.rectangle(input, new Point(rightDivider, 0), new Point(RECT_WIDTH, RECT_HEIGHT), new Scalar(255, 0, 0), 4);
        }

        // Draw Green Rectangle Depending on Region
        if (isWarehouse) {
            if (debug) {
                if (outputCase == Case.Left) {
                    Imgproc.rectangle(input, new Point(0, 0), new Point(leftDivider, RECT_HEIGHT), new Scalar(0, 255, 0), 4);
                } else if (outputCase == Case.Middle) {
                    Imgproc.rectangle(input, new Point(rightDivider, 0), new Point(RECT_WIDTH, RECT_HEIGHT), new Scalar(0, 255, 0), 4);
                }
            }
        } else {
            if (outputCase == Case.Middle) {
                Imgproc.rectangle(input, new Point(0, 0), new Point(leftDivider, RECT_HEIGHT), new Scalar(0, 255, 0), 4);
            } else if (outputCase == Case.Right) {
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
