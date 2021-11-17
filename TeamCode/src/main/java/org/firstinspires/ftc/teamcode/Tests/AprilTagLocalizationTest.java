package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpenCV.Vision;

@Autonomous
public class AprilTagLocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Vision detector = new Vision(this, Vision.Pipeline.AprilTag, 0.166);
        detector.start();

        waitForStart();

        detector.getAprilTagPipe().runAprilTag();
        double[] startMarkerPos = detector.getAprilTagPipe().getCenterOfMarker();

        while(opModeIsActive()){
            telemetry.addData("x", detector.getAprilTagPipe().localizeRobot(startMarkerPos)[0]);
            telemetry.addData("y", detector.getAprilTagPipe().localizeRobot(startMarkerPos)[1]);
            telemetry.addData("theta", detector.getAprilTagPipe().localizeRobot(startMarkerPos)[2]);
            telemetry.update();

            addPacket("x", detector.getAprilTagPipe().localizeRobot(startMarkerPos)[0]);
            addPacket("y", detector.getAprilTagPipe().localizeRobot(startMarkerPos)[1]);
            addPacket("theta", detector.getAprilTagPipe().localizeRobot(startMarkerPos)[2]);
            sendPacket();

            detector.getAprilTagPipe().runAprilTag();
        }
    }
}
