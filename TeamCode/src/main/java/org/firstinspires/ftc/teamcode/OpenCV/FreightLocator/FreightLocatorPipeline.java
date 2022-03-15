package org.firstinspires.ftc.teamcode.OpenCV.FreightLocator;

import static org.firstinspires.ftc.teamcode.OpenCV.FreightLocator.FreightLocator.maxX;
import static org.firstinspires.ftc.teamcode.OpenCV.FreightLocator.FreightLocator.maxY;
import static org.firstinspires.ftc.teamcode.OpenCV.FreightLocator.FreightLocator.minX;
import static org.firstinspires.ftc.teamcode.OpenCV.FreightLocator.FreightLocator.minY;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

@Config
public class FreightLocatorPipeline extends OpenCvPipeline {

    // Camera Constants
    public static double CAM_HEIGHT = 7.63;
    public static double CAM_FRONT = 8.5;
    public static double CAM_RIGHT = 5;
    public static double CAM_HEADING = Math.toRadians(10);
    public static double CAM_PHI = Math.toRadians(18.6);
    public static double CAM_VFOV = Math.toRadians(47.7);
    public static double CAM_HFOV = Math.toRadians(58.96);

    public static double MIN_AREA = 300;
    public static double MAX_AREA = 7000;

    // Image Processing Variables
    private FreightProcessor processor;
    private Mat[] output;
    private Rect white;
    private Rect yellow;

    // Freight ArrayLists
    private ArrayList<Freight> freights = new ArrayList<>();
    private ArrayList<Freight> prevFreights = new ArrayList<>();

    public static int RETURN_IMAGE = 0;

    public FreightLocatorPipeline() {
        processor = new FreightProcessor("locator");
    }

    @Override
    public Mat processFrame(Mat input) {
        // Process Image
        output = processor.processFrame(input);

        // Get White and Yellow Freights
        white = detectFreight(output[0]);
        yellow = detectFreight(output[1]);

        // Generate Final Detection
        if (white != null && yellow != null) {
            if (white.y + white.height > yellow.y + yellow.height) {
                annotateFreight(white, freights, input);
            } else {
                annotateFreight(yellow, freights, input);
            }
        } else if (white != null) {
            annotateFreight(white, freights, input);
        } else if (yellow != null) {
            annotateFreight(yellow, freights, input);
        } else {
            freights.add(new Freight(0, 0));
            log("No Freight Detected");
        }

        processor.saveMatToDisk("goated", input);

        prevFreights.clear();
        for (Freight freight : freights) {
            prevFreights.add(freight.clone());
        }
        freights.clear();

        if (1 <= RETURN_IMAGE && RETURN_IMAGE <= 3) {
            return output[RETURN_IMAGE - 1];
        } else {
            return input;
        }
    }

    // First stage freight detection
    private Rect detectFreight(Mat filtered) {
        // Find Contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(filtered, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);

        // Loop Through Contours
        ArrayList<Rect> rects = new ArrayList<>();
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (MIN_AREA < area && area < MAX_AREA) {
                rects.add(Imgproc.boundingRect(contour));
            }
        }

        // Find Bottommost Detection
        int bottom = 0;
        int index = -1;
        for (int i = 0; i < rects.size(); i++) {
            Rect rect = rects.get(i);
            if (rect.y + rect.height > bottom) {
                bottom = rect.y + rect.height;
                index = i;
            }
        }

        // Return if no Detection
        if (index == -1) {
            return null;
        }

        // Check if Detection is Single Freight
        Rect rect = rects.get(index);

        int x = rect.x;
        int y = rect.y;
        int w = rect.width;
        int h = rect.height;

        // TODO: fix eventually
        if (1 < w && w < 100 && 1 < h && h < 100) {
//        if (1 < w && w < 60 && 1 < h && h < 60) {
//            int[] center = {x + w / 2, y + h};
//            if (center[1] > 38 * center[0] / 11 - 475) {
            return rect;
//            } else {
//                return null;
//            }
        }

        // Draw Columns
        for (int i = 0; i < w / 20; i++) {
            Imgproc.rectangle(filtered, new Point(x + 20 * i, y), new Point(x + 20 * i + 20, y + h), new Scalar(0), 2);
        }

        // Rerun Detection Algorithm
        return detectFreights(x, y, filtered.submat(new Rect(x, y, w, h)));
    }

    // Second stage freight detection
    private Rect detectFreights(int x_, int y_, Mat filtered) {
        // Find Contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(filtered, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);

        // Loop Through Contours
        ArrayList<Rect> rects = new ArrayList<>();
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (100 < area && area < 1000) {
                Rect rect = Imgproc.boundingRect(contour);
                rect.x += x_;
                rect.y += y_;
                rects.add(rect);
            }
        }

        // Find Bottommost Detections
        int bottom = 0;
        ArrayList<Integer> indexes = new ArrayList<>();
        for (int i = 0; i < rects.size(); i++) {
            Rect rect = rects.get(i);
            if (bottom - 10 <= rect.y + rect.height && rect.y + rect.height <= bottom) {
                indexes.add(i);
            } else if (rect.y + rect.height > bottom) {
                bottom = rect.y + rect.height;
                indexes = new ArrayList<>(Collections.singletonList(i));
            }
        }

        // Return if no Detection
        if (indexes.size() == 0) {
            return null;
        }

        // Filter Detected Freights for Reflections
        rects = relativeFilter(rects); // TODO: make alliance dependent (leftmost vs rightmost)

        // Find Leftmost or Rightmost Freight
        int extreme = 0; // TODO: make alliance dependent (leftmost vs rightmost)
        int index = -1;
        for (int i = 0; i < rects.size(); i++) {
            Rect rect = rects.get(i);
            if (rect.x + rect.width / 2 > extreme) {
                extreme = rect.x + rect.width / 2;
                index = i;
            }
        }

        if (index == -1) {
            return null;
        } else {
            return rects.get(index);
        }
    }

    // Remove detections in wall reflection
    private ArrayList<Rect> relativeFilter(ArrayList<Rect> rects) {
        ArrayList<Rect> filteredRects = new ArrayList<>();

        // TODO: fix eventually
        for (Rect rect : rects) {
//            int[] center = {rect.x + rect.width / 2, rect.y + rect.height};
//            if (center[1] > 38 * center[0] / 11 - 475) {
            filteredRects.add(rect);
//            }
        }

        return filteredRects;
    }

    // Returns freight's position relative to camera
    private double[] map2Dto3D(double xPix, double yPix) {
//        return new double[] {xPix, yPix};
        double vfov = CAM_VFOV / 2;
        double hfov = CAM_HFOV / 2;
        double num = (Math.cos(-CAM_PHI - vfov)) / (2 * Math.sin(vfov)) + yPix * Math.sin(CAM_PHI);
        double den = (Math.sin(-CAM_PHI - vfov)) / (2 * Math.sin(vfov)) + yPix * Math.cos(CAM_PHI);
        double y = -CAM_HEIGHT * num / den;
        double x = Math.tan(hfov) * xPix * Math.hypot(CAM_HEIGHT, y);
        return new double[]{x + CAM_RIGHT, y + CAM_FRONT};
        // TODO: fix eventually
//        double s = Math.sin(CAM_HEADING);
//        double c = Math.cos(CAM_HEADING);
//        return new double[] {-s*x + c*y + CAM_RIGHT, c*x + s*y + CAM_FRONT};
    }

    // Draw freight on image
    private void annotateFreight(Rect detection, ArrayList<Freight> freights, Mat img) {
        // Annotate Image
        int x = detection.x;
        int y = detection.y;
        int w = detection.width / 2;
        int h = detection.height;
        x += w;
        w = (int) (0.214 * (y + h) - 28.450);

        Imgproc.rectangle(img, new Point(x - w, y + h - 2 * w), new Point(x + w, y + h), new Scalar(0, 255, 0), 2);
        Imgproc.circle(img, new Point(x, y + h), 3, new Scalar(255, 0, 0), -1);

        // Convert Freight Position
        double xPix = x / 120.0 - 1; // x ∈ [-1, 1]
        double yPix = 1 - y / 320.0; // y ∈ (0, 1)
        Freight curFreight = new Freight(map2Dto3D(xPix, yPix));

        // Save Freight Position
//        if (curFreight.getRelY() > 0) {
        freights.add(curFreight);
        log("(" + curFreight.getRelX() + ", " + curFreight.getRelY() + ") " + curFreight.getRelDist());
//        }
    }

    // Return raw freight
    public ArrayList<Freight> getRawFreight() {
        return new ArrayList<>(prevFreights);
    }

    // Return a list of coordinate-filtered freight
    public ArrayList<Freight> getFilteredFreight(double robotX, double robotY, double robotTheta, double minX, double minY, double maxX, double maxY) {
        ArrayList<Freight> freights = getRawFreight();

        // Calculate absolute coordinates
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
