package org.firstinspires.ftc.teamcode.Debug;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.OpenCV.Freight;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;
import org.firstinspires.ftc.teamcode.RobotClasses.Turret;

public class Dashboard {
    public static FtcDashboard dashboard = FtcDashboard.getInstance();
    public static TelemetryPacket packet = new TelemetryPacket();

    public static void drawRobot(Robot robot) {
        drawRobot(robot, "grey");
    }

    public static void drawRobot(Robot robot, String drivetrainColor) {
        drawRobot(robot.x, robot.y, robot.theta, !robot.intake.slidesIsHome(), robot.deposit.getSlidesDistInches(), robot.turret.getTurretTheta(), drivetrainColor);
    }

    public static void drawRobot(double robotX, double robotY, double robotTheta, boolean intakeSlidesExtend, double depositSlidesDist, double turretTheta, String drivetrainColor) {
        drawDrivetrain(robotX, robotY, robotTheta, drivetrainColor);
        drawIntakeSlides(robotX, robotY, robotTheta, intakeSlidesExtend);
        drawDepositTurretSlides(robotX, robotY, robotTheta, turretTheta, depositSlidesDist);
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

    public static void drawIntakeSlides(double x, double y, double theta, boolean extended) {
        double extendedPos;
        if (extended){
            extendedPos = 13.5;
        } else {
            extendedPos = 0;
        }
        double[] leftX = {-2.5 * cos(theta) - 9 * sin(theta) + x, -2.5 * cos(theta) - (9 + extendedPos) * sin(theta) + x, -3 * cos(theta) - (9 + extendedPos) * sin(theta) + x, -3 * cos(theta) - 9 * sin(theta) + x};
        double[] leftY = {-2.5 * sin(theta) + 9 * cos(theta) + y, -2.5 * sin(theta) + (9 + extendedPos) * cos(theta) + y, -3 * sin(theta) + (9 + extendedPos) * cos(theta) + y, -3 * sin(theta) + 9 * cos(theta) + y};

        double[] rightX = {2.5 * cos(theta) - 9 * sin(theta) + x, 2.5 * cos(theta) - (9 + extendedPos) * sin(theta) + x, 3 * cos(theta) - (9 + extendedPos) * sin(theta) + x, 3 * cos(theta) - 9 * sin(theta) + x};
        double[] rightY = {2.5 * sin(theta) + 9 * cos(theta) + y, 2.5 * sin(theta) + (9 + extendedPos) * cos(theta) + y, 3 * sin(theta) + (9 + extendedPos) * cos(theta) + y, 3 * sin(theta) + 9 * cos(theta) + y};

        drawPolygon(leftX, leftY, "orange");
        drawPolygon(rightX, rightY, "orange");
    }

    public static void drawDepositTurretSlides(double x, double y, double robotTheta, double turretTheta, double slidesDist) {
        double extendedPos = 11.5 + slidesDist;
        double theta = robotTheta + turretTheta;
        double turretCenterX = x + Turret.TURRET_Y_OFFSET * cos(robotTheta);
        double turretCenterY = y + Turret.TURRET_Y_OFFSET * sin(robotTheta);

        double[] leftSlidesX = {-2 * cos(theta) - -4.5 * sin(theta) + turretCenterX, -2 * cos(theta) - extendedPos * sin(theta) + turretCenterX, -3.5 * cos(theta) - extendedPos * sin(theta) + turretCenterX, -3.5 * cos(theta) - -4.5 * sin(theta) + turretCenterX};
        double[] leftSlidesY = {-2 * sin(theta) + -4.5 * cos(theta) + turretCenterY, -2 * sin(theta) + extendedPos * cos(theta) + turretCenterY, -3.5 * sin(theta) + extendedPos * cos(theta) + turretCenterY, -3.5 * sin(theta) + -4.5 * cos(theta) + turretCenterY};

        double[] rightSlidesX = {2 * cos(theta) - -4.5 * sin(theta) + turretCenterX, 2 * cos(theta) - extendedPos * sin(theta) + turretCenterX, 3.5 * cos(theta) - extendedPos * sin(theta) + turretCenterX, 3.5 * cos(theta) - -4.5 * sin(theta) + turretCenterX};
        double[] rightSlidesY = {2 * sin(theta) + -4.5 * cos(theta) + turretCenterY, 2 * sin(theta) + extendedPos * cos(theta) + turretCenterY, 3.5 * sin(theta) + extendedPos * cos(theta) + turretCenterY, 3.5 * sin(theta) + -4.5 * cos(theta) + turretCenterY};

        drawPolygon(leftSlidesX, leftSlidesY, "red");
        drawPolygon(rightSlidesX, rightSlidesY, "red");
    }

    public static void drawFreight(Freight freight, String color) {}

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
