package org.firstinspires.ftc.teamcode.Debug;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.OpenCV.FreightLocator.Freight;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

public class Dashboard {
    public static FtcDashboard dashboard = FtcDashboard.getInstance();
    public static TelemetryPacket packet = new TelemetryPacket();

    public static void drawRobot(Robot robot) {
        drawRobot(robot, "grey");
    }

    public static void drawRobot(Robot robot, String drivetrainColor) {
        drawRobot(robot.x, robot.y, robot.theta, robot.intake.getSlidesPos(), robot.deposit.getSlidesPos(), robot.deposit.isArmOut(), drivetrainColor);
    }

    public static void drawRobot(double robotX, double robotY, double robotTheta, double intakeSlidePos, double depositSlidesDist, boolean isArmExtended, String drivetrainColor) {
        drawDrivetrain(robotX, robotY, robotTheta, drivetrainColor);
//        drawDepositSlides(robotX, robotY, robotTheta, depositSlidesDist);
        drawIntakeSlides(robotX, robotY, robotTheta, intakeSlidePos);
    }

    public static void drawDrivetrain(double robotX, double robotY, double robotTheta, String color) {
        double x = robotY - 72;
        double y = 72 - robotX;
        double theta = robotTheta;

        double[] xcoords = {-6.5 * cos(theta) - 9 * sin(theta) + x, 6.5 * cos(theta) - 9 * sin(theta) + x, 6.5 * cos(theta) + 9 * sin(theta) + x, -6.5 * cos(theta) + 9 * sin(theta) + x};
        double[] ycoords = {-6.5 * sin(theta) + 9 * cos(theta) + y, 6.5 * sin(theta) + 9 * cos(theta) + y, 6.5 * sin(theta) - 9 * cos(theta) + y, -6.5 * sin(theta) - 9 * cos(theta) + y};

        packet.fieldOverlay().setFill(color).fillPolygon(xcoords, ycoords);
        packet.fieldOverlay().setFill("black").fillCircle(6.5 * sin(theta) + x,  -6.5 * cos(theta) + y, 2.25);
    }

    public static void drawIntakeSlides(double x, double y, double theta, double slidesPos) {
        double extendedPos =  slidesPos * 0.052;
        theta -= PI / 2;

        double[] leftX = {-3.5 * cos(theta) - (9 + extendedPos) * sin(theta) + x, -3 * cos(theta) - (9 + extendedPos) * sin(theta) + x, -3 * cos(theta) + 8.5 * sin(theta) + x, -3.5 * cos(theta) + 8.5 * sin(theta) + x};
        double[] leftY = {-3.5 * sin(theta) + (9 + extendedPos) * cos(theta) + y, -3 * sin(theta) + (9 + extendedPos) * cos(theta) + y, -3 * sin(theta) - 8.5 * cos(theta) + y, -3.5 * sin(theta) - 8.5 * cos(theta) + y};

        double[] rightX = {3 * cos(theta) - (9 + extendedPos) * sin(theta) + x, 3.5 * cos(theta) - (9 + extendedPos) * sin(theta) + x, 3.5 * cos(theta) + 8.5 * sin(theta) + x, 3 * cos(theta) + 8.5 * sin(theta) + x};
        double[] rightY = {3 * sin(theta) + (9 + extendedPos) * cos(theta) + y, 3.5 * sin(theta) + (9 + extendedPos) * cos(theta) + y, 3.5 * sin(theta) - 8.5 * cos(theta) + y, 3 * sin(theta) - 8.5 * cos(theta) + y};

        if (slidesPos <= 70) {
            drawPolygon(leftX, leftY, "green");
            drawPolygon(rightX, rightY, "green");
        } else if (slidesPos >= 285) {
            drawPolygon(leftX, leftY, "red");
            drawPolygon(rightX, rightY, "red");
        } else {
            drawPolygon(leftX, leftY, "yellow");
            drawPolygon(rightX, rightY, "yellow");
        }
    }

    public static void drawDepositSlides(double x, double y, double theta, double slidesPos) {
        double extendedPos = slidesPos * 0.045;
        theta -= PI / 2;

        double[] leftX = {-3 * cos(theta) + x, -4.5 * cos(theta) + x, -4.5 * cos(theta) + (9 + extendedPos) * sin(theta) + x, -3 * cos(theta) + (9 + extendedPos) * sin(theta) + x};
        double[] leftY = {-3 * sin(theta) + y, -4.5 * sin(theta) + y, -4.5 * sin(theta) - (9 + extendedPos) * cos(theta) + y, -3 * sin(theta) - (9 + extendedPos) * cos(theta) + y};

        double[] rightX = {3 * cos(theta) + x, 4.5 * cos(theta) + x, 4.5 * cos(theta) + (9 + extendedPos) * sin(theta) + x, 3 * cos(theta) + (9 + extendedPos) * sin(theta) + x};
        double[] rightY = {3 * sin(theta) + y, 4.5 * sin(theta) + y, 4.5 * sin(theta) - (9 + extendedPos) * cos(theta) + y, 3 * sin(theta) - (9 + extendedPos) * cos(theta) + y};

        if (slidesPos <= 20) {
            drawPolygon(leftX, leftY, "green");
            drawPolygon(rightX, rightY, "green");
        } else if (slidesPos >= 495) {
            drawPolygon(leftX, leftY, "red");
            drawPolygon(rightX, rightY, "red");
        } else {
            drawPolygon(leftX, leftY, "yellow");
            drawPolygon(rightX, rightY, "yellow");
        }
    }

    public static void drawFreight(Freight freight, String color) {
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

    public double[] rotatePoint(double x, double y, double theta) {
        return new double[]{cos(x)-sin(y), sin(x)+cos(y)};
    }

    //rotate a rectangle about origin
    public double[][] rotateRect(double x1, double y1, double x2, double y2, double theta){
        double[][] rotatedPoints = new double[2][2];
        rotatedPoints[0] = rotatePoint(x1, y1, theta);
        rotatedPoints[1] = rotatePoint(x2, y2, theta);
        return rotatedPoints;
    }
}
