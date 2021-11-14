package org.firstinspires.ftc.teamcode.Debug;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import static java.lang.Math.sin;
import static java.lang.Math.cos;

public class Dashboard {
    public static FtcDashboard dashboard = FtcDashboard.getInstance();
    public static TelemetryPacket packet = new TelemetryPacket();

    public static void drawRobot(Robot robot) {
        drawRobot(robot, "grey");
    }

    public static void drawRobot(Robot robot, String drivetrainColor) {
        drawRobot(robot.x, robot.y, robot.theta, robot.deposit.getSlidesHeight(), drivetrainColor);
    }

    public static void drawRobot(double robotX, double robotY, double robotTheta, double slidesPosition, String drivetrainColor) {
        drawDrivetrain(robotX, robotY, robotTheta, drivetrainColor);
        drawSlides(robotX, robotY, robotTheta, slidesPosition);
    }

    public static void drawDrivetrain(double robotX, double robotY, double robotTheta, String color) {
        double x = robotY - 72;
        double y = 72 - robotX;
        double theta = robotTheta;

        double[] xcoords = {-6.5 * cos(theta) - 9 * sin(theta) + x, 6.5 * cos(theta) - 9 * sin(theta) + x, 6.5 * cos(theta) + 9 * sin(theta) + x, -6.5 * cos(theta) + 9 * sin(theta) + x};
        double[] ycoords = {-6.5 * sin(theta) + 9 * cos(theta) + y, 6.5 * sin(theta) + 9 * cos(theta) + y, 6.5 * sin(theta) - 9 * cos(theta) + y, -6.5 * sin(theta) - 9 * cos(theta) + y};

        packet.fieldOverlay().setFill(color).fillPolygon(xcoords, ycoords);
        packet.fieldOverlay().setFill("green").fillCircle(-4.5 * cos(theta) + 6.5 * sin(theta) + x, -4.5 * sin(theta) - 6.5 * cos(theta) + y, 2.25);
    }

    public static void drawSlides(double x, double y, double theta, double position) {
        double[] leftX = {-4 * cos(theta) - 9 * sin(theta) + x, -2.5 * cos(theta) - 9 * sin(theta) + x, -2.5 * cos(theta) - (9 + position) * sin(theta) + x, -4 * cos(theta) - (9 + position) * sin(theta) + x};
        double[] leftY = {4 * sin(theta) - 9 * cos(theta) + y, 2.5 * sin(theta) - 9 * cos(theta) + y, 2.5 * sin(theta) - (9 + position) * cos(theta) + y, 4 * sin(theta) - (9 + position) * cos(theta) + y};

        double[] rightX = {4 * cos(theta) - 9 * sin(theta) + x, 2.5 * cos(theta) - 9 * sin(theta) + x, 2.5 * cos(theta) - (9 + position) * sin(theta) + x, 4 * cos(theta) - (9 + position) * sin(theta) + x};
        double[] rightY = {-4 * sin(theta) - 9 * cos(theta) + y, -2.5 * sin(theta) - 9 * cos(theta) + y, -2.5 * sin(theta) - (9 + position) * cos(theta) + y, -4 * sin(theta) - (9 + position) * cos(theta) + y};

        drawPolygon(leftX, leftY, "grey");
        drawPolygon(rightX, rightY, "grey");
    }

    public static void drawField() {
        outlineRect(0, 0, 144, 144, "black");
    }

    public static void drawPoint(double x, double y, String color) {
        packet.fieldOverlay().setFill(color).fillCircle(y - 72, 72 - x, 0.5);
    }

    public static void drawLine(double x1, double y1, double x2, double y2, String color) {
        packet.fieldOverlay().setStroke(color).strokeLine(y1 - 72, 72 - x1, y2 - 72, 72 - x2);
    }

    public static void outlineRect(double x1, double y1, double x2, double y2, String color) {
        double[] xcoords = {y1 - 72, y2 - 72, y2 - 72, y1 - 72};
        double[] ycoords = {72 - x1, 72 - x1, 72 - x2, 72 - x2};
        packet.fieldOverlay().setStroke(color).strokePolygon(xcoords, ycoords);
    }

    public static void drawRect(double x1, double y1, double x2, double y2, String color) {
        double[] xcoords = {y1 - 72, y2 - 72, y2 - 72, y1 - 72};
        double[] ycoords = {72 - x1, 72 - x1, 72 - x2, 72 - x2};
        packet.fieldOverlay().setFill(color).fillPolygon(xcoords, ycoords);
    }

    public static void drawPolygon(double[] x, double[] y, String color) {
        double[] xcoords = new double[y.length];
        double[] ycoords = new double[x.length];

        for (int i = 0; i < xcoords.length; i++) {
            xcoords[i] = y[i] - 72;
        }

        for (int i = 0; i < ycoords.length; i++) {
            ycoords[i] = 72 - x[i];
        }

        packet.fieldOverlay().setFill(color).fillPolygon(xcoords, ycoords);
    }

    public static void addPacket(String key, Object value) {
        packet.put(key, value.toString());
    }

    public static void sendPacket() {
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }
}
