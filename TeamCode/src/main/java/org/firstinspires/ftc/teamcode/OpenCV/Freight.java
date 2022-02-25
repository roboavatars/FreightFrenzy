package org.firstinspires.ftc.teamcode.OpenCV;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.teamcode.RobotClasses.Turret;

import java.util.ArrayList;
import java.util.Comparator;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static org.firstinspires.ftc.teamcode.OpenCV.FreightLocator.FreightLocator.maxX;
import static org.firstinspires.ftc.teamcode.OpenCV.FreightLocator.FreightLocator.maxY;
import static org.firstinspires.ftc.teamcode.OpenCV.FreightLocator.FreightLocator.minX;
import static org.firstinspires.ftc.teamcode.OpenCV.FreightLocator.FreightLocator.minY;

public class Freight {
    private double relX;
    private double relY;
    private double absX;
    private double absY;

    private static double DIST_THRESH = 1.5;

    public Freight(double relX, double relY, double absX, double absY) {
        this.relX = relX;
        this.relY = relY;
        this.absX = absX;
        this.absY = absY;
    }

    public Freight(double relX, double relY) {
        this.relX = relX;
        this.relY = relY;
    }

    public Freight(double startX, double startY, double theta) {
        this.absX = startX + Turret.TURRET_Y_OFFSET * cos(theta);
        this.absY = startY + Turret.TURRET_Y_OFFSET * sin(theta);
    }

    // Return a sorted list with up to three coordinate-filtered freights
    public static ArrayList<Freight> getFreightCoords(ArrayList<Freight> freights, double minX, double minY, double maxX, double maxY, double robotX, double robotY) {
        // Remove freights out of bounds
        int i = 0;
        while (i < freights.size()) {
            Freight freight = freights.get(i);
            if (freight.getX() < minX || freight.getX() > maxX || freight.getY() < minY || freight.getY() > maxY) {
                freights.remove(i);
            } else {
                i++;
            }
        }

        // Remove freights that are too close to each other
        i = 0;
        while (i < freights.size()) {
            double xi = freights.get(i).getX();
            double yi = freights.get(i).getY();
            int j = i + 1;

            while (j < freights.size()) {
                Freight freight = freights.get(j);
                if (Math.abs(xi - freight.getX()) < DIST_THRESH && Math.abs(yi - freight.getY()) < DIST_THRESH) {
                    freights.remove(j);
                } else {
                    j++;
                }
            }
            i++;
        }

        // Sort freights based on y coordinate
        freights.sort(Comparator.comparingDouble(r -> r.getY()));

        // Return up to three freights
        if (freights.size() > 3) {
            freights = new ArrayList<>(freights.subList(0, 3));
        }

        // Determine left or right sweep
        if (freights.size() > 0) {
            if (freights.get(freights.size() - 1).getY() - freights.get(0).getY() > 8) {
                Freight closest = freights.remove(0);
                if (closest.getX() <= robotX + 9) {
                    freights.sort(Comparator.comparingDouble(r -> r.getX()));
                } else {
                    freights.sort(Comparator.comparingDouble(r -> -r.getX()));
                }
                freights.add(0, closest);
            } else {
                freights.sort(Comparator.comparingDouble(r -> r.getX()));
            }
        }

        return freights;
    }

    // Return a sorted list with up to three coordinate-filtered freights
    public static ArrayList<Freight> getFreightCoords(ArrayList<Freight> freights, double robotX, double robotY) {
        return getFreightCoords(freights, minX, minY, maxX, maxY, robotX, robotY);
    }

    // Calculate freight absolute coordinates using relative coordinates and robot position
    public void calcAbsCoords(double robotX, double robotY, double robotTheta) {
        absX = robotX + relX * sin(robotTheta) + relY * cos(robotTheta);
        absY = robotY - relX * cos(robotTheta) + relY * sin(robotTheta);
    }

    @SuppressLint("DefaultLocale")
    public String toString() {
        return "R(" + String.format("%.3f", relX) + ", " + String.format("%.3f", relY) + "), A(" + String.format("%.3f", absX) + ", " + String.format("%.3f", absY) + ")";
    }

    public Freight clone() {
        return new Freight(relX, relY, absX, absY);
    }

    public double[] driveToFreight(double robotX, double robotY) {
        return new double[] {absX, absY, Math.atan2(absY - robotY, absX - robotX)};
    }

    public double[] getAbsCoords() {
        return new double[] {absX, absY};
    }

    public double getRelDist() {
        return Math.hypot(relX, relY);
    }

    public double getAbsDist(double robotX, double robotY) {
        return Math.hypot(absX - robotX, absY - robotY);
    }

    public double getRelX() {
        return relX;
    }

    public double getRelY() {
        return relY;
    }

    public double getX() {
        return absX;
    }

    public double getY() {
        return absY;
    }
}
