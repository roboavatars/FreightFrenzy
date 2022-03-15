package org.firstinspires.ftc.teamcode.OpenCV.TapeDetector;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class TapeDetectorPipeline extends OpenCvPipeline {

    // Camera Constants
    public static double CAM_HEIGHT = 8.5;
    public static double CAM_FRONT = 8.5;
    public static double CAM_RIGHT = 5;
    public static double CAM_HEADING = Math.toRadians(10);
    public static double CAM_PHI = Math.toRadians(13);
    public static double CAM_VFOV = Math.toRadians(47.7);
    public static double CAM_HFOV = Math.toRadians(58);

    // CV Thresholds
    public static int MIN_H = 0;
    public static int MIN_S = 0;
    public static int MIN_V = 210;
    public static int MAX_H = 255;
    public static int MAX_S = 30;
    public static int MAX_V = 255;

    public static double AREA_MIN = 1000;
    public static double AREA_MAX = 8000;
    public static double HW_MIN = 4;

    // Image Cropping
    public static int RETURN_IMAGE = 0;
    public static boolean debug = true;
    private double[] output = new double[]{0, 0};

    // Image Processing Mats
    private Mat hsv = new Mat();
    private Mat processed = new Mat();
    private Mat mask = new Mat();
    private Mat save;

    private String path = "/sdcard/OpenCV/tape/";

    public TapeDetectorPipeline() {
    }

    @Override
    public Mat processFrame(Mat input) {
        // Convert to HSV Color Space
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
        Core.inRange(hsv, new Scalar(MIN_H, MIN_S, MIN_V), new Scalar(MAX_H, MAX_S, MAX_V), processed);

        // Remove Noise
        Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_OPEN, new Mat());
        Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE, new Mat());

        // Mask Image for Debug
        if (debug) {
            Core.bitwise_and(input, input, mask, processed);
        }

        // Find Contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(processed, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);

        // Loop Through Contours
        Point bestCenter = new Point(0, 0);
        Rect bestRect = null;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (AREA_MIN < area && area < AREA_MAX) {
                Rect rect = Imgproc.boundingRect(contour);
                if ((double) rect.height / rect.width > HW_MIN) {
                    Point center = new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);

                    if (center.y > bestCenter.y) {
                        bestCenter = center;
                        bestRect = rect;
                    }
                }
            }
        }

        if (bestRect != null) {
            Imgproc.rectangle(input, bestRect, new Scalar(0, 255, 0), 3);
            Imgproc.circle(input, bestCenter, 5, new Scalar(0, 255, 0), -1);
            output = map2Dto3D(bestCenter.x / 120.0 - 1, 1 - bestCenter.y / 320.0);
        } else {
            output = new double[]{0, 0};
        }

//        // Draw Green Rectangle Depending on Region
//        if (debug) {
//            saveMatToDisk("input", input);
//            saveMatToDisk("mask", mask);
//            saveMatToDisk("greenRect", input);
//        }

        log("Output: (" + output[0] + ", " + output[1] + ")");

        if (RETURN_IMAGE == 1) {
            return processed;
        } else if (RETURN_IMAGE == 2) {
            return mask;
        } else {
            return input;
        }
    }

    public double[] getResult() {
        return output;
    }

    // Returns point's position relative to camera
    private double[] map2Dto3D(double xPix, double yPix) {
//        return new double[] {xPix, yPix};
        double vfov = CAM_VFOV / 2;
        double hfov = CAM_HFOV / 2;
        double num = (Math.cos(-CAM_PHI - vfov)) / (2 * Math.sin(vfov)) + yPix * Math.sin(CAM_PHI);
        double den = (Math.sin(-CAM_PHI - vfov)) / (2 * Math.sin(vfov)) + yPix * Math.cos(CAM_PHI);
        double y = -CAM_HEIGHT * num / den;
        double x = Math.tan(hfov) * xPix * Math.hypot(CAM_HEIGHT, y);
        double s = Math.sin(CAM_HEADING);
        double c = Math.cos(CAM_HEADING);
        return new double[]{-s * x + c * y + CAM_RIGHT, c * x + s * y + CAM_FRONT};
    }

    private void log(String message) {
        Log.w("tape-detector-pipe", message);
    }

    public void saveMatToDisk(String name, Mat mat) {
        save = mat.clone();
        Imgproc.cvtColor(mat, save, Imgproc.COLOR_BGR2RGB);
        Imgcodecs.imwrite(path + name + ".jpg", save);
    }
}
