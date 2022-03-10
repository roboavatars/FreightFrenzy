package org.firstinspires.ftc.teamcode.OpenCV.FreightLocator;

import static org.firstinspires.ftc.teamcode.OpenCV.FreightLocator.FreightLocator.maxX;
import static org.firstinspires.ftc.teamcode.OpenCV.FreightLocator.FreightLocator.maxY;
import static org.firstinspires.ftc.teamcode.OpenCV.FreightLocator.FreightLocator.minX;
import static org.firstinspires.ftc.teamcode.OpenCV.FreightLocator.FreightLocator.minY;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

@Config
public class FreightLocatorPipeline extends OpenCvPipeline {

    // CV Thresholds
    public static double HEIGHT_MIN = 15;
    public static double HEIGHT_MAX = 130;
    public static double WIDTH_MIN = 3;
    public static double WIDTH_MAX = 75;
    public static double ANGLE_MIN = 70;
    public static double ANGLE_MAX = 110;

    // Camera Constants
    public static double CAM_HEIGHT = 9;
    public static double CAM_FRONT = 9;
    public static double CAM_RIGHT = 8;
    public static double CAM_PHI = Math.toRadians(-0.13);
    public static double CAM_VFOV = Math.toRadians(47.0);
    public static double CAM_HFOV = Math.toRadians(56.2);

    // Image Processing Mats
    private FreightProcessor processor;
    private Mat processed = new Mat();

    // Ellipse Variables
    private double width;
    private double height;
    private double angle;
    private double xPix;
    private double yPix;

    // Freight Position Variables
    private ArrayList<Freight> freights = new ArrayList<>();
    private ArrayList<Freight> prevFreights = new ArrayList<>();

    public FreightLocatorPipeline() {
        processor = new FreightProcessor("locator");
    }

    @Override
    public Mat processFrame(Mat input) {
        // Process Image
        processed = processor.processFrame(input)[0];

        // Find Contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(processed, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        // Loop Through Contours
        for (MatOfPoint contour : contours) {
            Point[] contourArray = contour.toArray();

            // Bound Ellipse if Contour is Large Enough
            if (contourArray.length >= 5) {
                MatOfPoint2f areaPoints = new MatOfPoint2f(contourArray);
                RotatedRect ellipse = Imgproc.fitEllipse(areaPoints);

                // Save Ellipse Data
                width = ellipse.size.width;
                height = ellipse.size.height;
                angle = ellipse.angle;
                xPix = ellipse.center.x / 160 - 1; // x ∈ [-1, 1]
                yPix = 1 - ellipse.center.y / 240; // y ∈ (0, 1)

//                log("x: " + xPix);
//                log("y: " + yPix);
//                log("width: " + width);
//                log("height: " + height);

                // Analyze Valid Contours
                if (HEIGHT_MIN < height && height < HEIGHT_MAX && WIDTH_MIN < width && width < WIDTH_MAX && ANGLE_MIN < angle && angle < ANGLE_MAX) {

                    // Calculate Center of Freight if Y is in Valid Domain
                    if (0 < yPix && yPix < 0.37) {
                        Imgproc.ellipse(input, ellipse, new Scalar(0, 255, 0), 1);

                        double[] xy = map2Dto3D(xPix, yPix);
                        Freight curFreight = new Freight(xy[0], xy[1]);

                        log("width, height, angle: " + width + ", " + height + ", " + angle);
                        log("(" + curFreight.getRelX() + ", " + curFreight.getRelY() + ") " + curFreight.getRelDist());

                        // Save Freight Position
                        if (curFreight.getRelY() > 0) {
                            freights.add(curFreight);
                        }
                    }
                } else {
//                     Imgproc.ellipse(input, ellipse, new Scalar(255, 0, 0), 1);
                }
            }
        }
        processor.saveMatToDisk("ellipse.jpg", input);

        // Return (0, 0) If No Freight Detected
        if (freights.size() == 0) {
            freights.add(new Freight(0, 0));
            log("No Freight Detected");
        }

        prevFreights.clear();
        for (Freight freight : freights) {
            prevFreights.add(freight.clone());
        }
        freights.clear();

        return input;
    }

    // Returns freight's position relative to camera
    private double[] map2Dto3D(double xPix, double yPix) {
        double vfov = CAM_VFOV / 2;
        double hfov = CAM_HFOV / 2;
        double num = (Math.cos(-CAM_PHI - vfov)) / (2 * Math.sin(vfov)) + yPix * Math.sin(CAM_PHI);
        double den = (Math.sin(-CAM_PHI - vfov)) / (2 * Math.sin(vfov)) + yPix * Math.cos(CAM_PHI);
        double y = -CAM_HEIGHT * num / den;
        return new double[] {Math.tan(hfov) * xPix * Math.hypot(CAM_HEIGHT, y) + CAM_RIGHT, y + CAM_FRONT};
    }

    // Return freights
    public ArrayList<Freight> getRawFreights() {
        return new ArrayList<>(prevFreights);
    }

    // Return a list of coordinate-filtered freight
    public ArrayList<Freight> getFilteredFreight(double robotX, double robotY, double robotTheta, double minX, double minY, double maxX, double maxY) {
        ArrayList<Freight> freights = getRawFreights();
        int i = 0;
        while (i < freights.size()) {
            try {
                freights.get(i).calcAbsCoords(robotX, robotY, robotTheta);
                i++;
            } catch (NullPointerException e) {
                freights.remove(i);
            }
        }

        // Remove freights out of bounds
        i = 0;
        while (i < freights.size()) {
            Freight freight = freights.get(i);
            if (freight.getX() < minX || freight.getX() > maxX || freight.getY() < minY || freight.getY() > maxY) {
                freights.remove(i);
            } else {
                i++;
            }
        }

        // Sort freights based on y coordinate
        freights.sort(Comparator.comparingDouble(Freight::getY));

        // Return freight
        if (freights.size() > 1) {
            return new ArrayList<>(freights.subList(0, 1));
        } else {
            return freights;
        }
    }

    public ArrayList<Freight> getFilteredFreight(double robotX, double robotY, double robotTheta) {
        return getFilteredFreight(robotX, robotY, robotTheta, minX, minY, maxX, maxY);
    }

    public void log(String message) {
        Log.w("freight-locator-pipe", message);
    }
}
