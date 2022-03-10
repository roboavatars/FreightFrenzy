package org.firstinspires.ftc.teamcode.OpenCV.FreightLocator;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

@Config
@SuppressLint("SdCardPath")
public class FreightProcessor {

    // Color Filtering Thresholds
    public static double minHWhite = 0;
    public static double minSWhite = 0;
    public static double minVWhite = 167;
    public static double maxHWhite = 177;
    public static double maxSWhite = 66;
    public static double maxVWhite = 255;

    public static int i = 0;

    public static double minHYellow = 16;
    public static double minSYellow = 85;
    public static double minVYellow = 139;
    public static double maxHYellow = 32;
    public static double maxSYellow = 255;
    public static double maxVYellow = 255;

    // Image Processing Mats
    private Mat hsv = new Mat();
    private Mat filteredWhite = new Mat();
    private Mat filteredYellow = new Mat();
    private Mat filtered = new Mat();
    private Mat mask = new Mat();
    private Mat save;

    private String path = "/sdcard/EasyOpenCV/";

    public FreightProcessor(String prefix) {
        path += prefix + '-';
    }

    public Mat[] processFrame(Mat input) {
        // Convert to HSV Color Space
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
        Core.inRange(hsv, new Scalar(minHWhite, minSWhite, minVWhite), new Scalar(maxHWhite, maxSWhite, maxVWhite), filteredWhite);
        Core.inRange(hsv, new Scalar(minHYellow, minSYellow, minVYellow), new Scalar(maxHYellow, maxSYellow, maxVYellow), filteredYellow);

        // Remove Noise
        Imgproc.morphologyEx(filteredWhite, filteredWhite, Imgproc.MORPH_OPEN, new Mat());
        Imgproc.morphologyEx(filteredWhite, filteredWhite, Imgproc.MORPH_CLOSE, new Mat());
        Imgproc.morphologyEx(filteredYellow, filteredYellow, Imgproc.MORPH_OPEN, new Mat());
        Imgproc.morphologyEx(filteredYellow, filteredYellow, Imgproc.MORPH_CLOSE, new Mat());

        // Mask Image for Debug
        Core.bitwise_or(filteredWhite, filteredYellow, filtered);
        Core.bitwise_and(input, input, mask, filtered);

        // Save Images for Debug
        saveMatToDisk("input", input);
        saveMatToDisk("filteredWhite", filteredWhite);
        saveMatToDisk("filteredYellow", filteredYellow);
        saveMatToDisk("filtered", filtered);
        saveMatToDisk("mask", mask);

        i++;

        return new Mat[] {filteredWhite, filteredYellow, filtered, mask};
    }

    public void saveMatToDisk(String name, Mat mat) {
        save = mat.clone(); // TODO: test if this is even necessary
        Imgproc.cvtColor(mat, save, Imgproc.COLOR_BGR2RGB);
        Imgcodecs.imwrite(path + name + i + ".jpg", save);
    }
}
