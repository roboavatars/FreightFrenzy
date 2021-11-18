package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawRect;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawDrivetrain;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpenCV.Vision;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@Autonomous
public class AprilTagLocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, 63 , 135, PI/2 , true, true);

        Vision detector = new Vision(this, Vision.Pipeline.AprilTag);
        detector.start();

        waitForStart();

        detector.getAprilTagPipe().runAprilTag();
        double[] startMarkerPos = detector.getAprilTagPipe().getCenterOfMarker();

        startMarkerPos[0] += robot.x;
        startMarkerPos[1] += robot.y;
        startMarkerPos[2] += robot.theta;

        while (opModeIsActive()) {
            telemetry.addData("x", detector.getAprilTagPipe().localizeRobot(startMarkerPos)[0]);
            telemetry.addData("y", detector.getAprilTagPipe().localizeRobot(startMarkerPos)[1]);
            telemetry.addData("theta", detector.getAprilTagPipe().localizeRobot(startMarkerPos)[2]);
            telemetry.update();

            addPacket("x", detector.getAprilTagPipe().localizeRobot(startMarkerPos)[0]);
            addPacket("y", detector.getAprilTagPipe().localizeRobot(startMarkerPos)[1]);
            addPacket("theta", detector.getAprilTagPipe().localizeRobot(startMarkerPos)[2]);
            sendPacket();

            //april tag based drivetrain
            drawDrivetrain(detector.getAprilTagPipe().localizeRobot(startMarkerPos)[0], detector.getAprilTagPipe().localizeRobot(startMarkerPos)[1], detector.getAprilTagPipe().localizeRobot(startMarkerPos)[2], "blue");

            //team marker
            double[] xcoords = {-2 * cos(startMarkerPos[2]) - 2 * sin(startMarkerPos[2]) + startMarkerPos[0], 2 * cos(startMarkerPos[2]) + 9 * sin(startMarkerPos[2]) + startMarkerPos[0]};
            double[] ycoords = {-2 * sin(startMarkerPos[2]) + 2 * cos(startMarkerPos[2]) + startMarkerPos[1], 2 * sin(startMarkerPos[2]) - 9 * cos(startMarkerPos[2]) + startMarkerPos[1]};
            drawRect(xcoords[0], ycoords[0], xcoords[1], ycoords[1], "green");
            detector.getAprilTagPipe().runAprilTag();
            robot.update();
        }
    }
}
