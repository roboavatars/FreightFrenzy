package org.firstinspires.ftc.teamcode.OpenCV;

import static org.firstinspires.ftc.teamcode.OpenCV.FreightLocator.FreightLocator.maxX;
import static org.firstinspires.ftc.teamcode.OpenCV.FreightLocator.FreightLocator.maxY;
import static org.firstinspires.ftc.teamcode.OpenCV.FreightLocator.FreightLocator.minX;
import static org.firstinspires.ftc.teamcode.OpenCV.FreightLocator.FreightLocator.minY;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.teamcode.RobotClasses.Turret;

import java.util.ArrayList;
import java.util.Comparator;

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

    // Return a list of coordinate-filtered freight
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

        // Sort freights based on y coordinate
        freights.sort(Comparator.comparingDouble(Freight::getY));

        // Return freight
        if (freights.size() > 1) {
            return new ArrayList<>(freights.subList(0, 1));
        } else {
            return freights;
        }
    }

    // Return a list of coordinate-filtered freight
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