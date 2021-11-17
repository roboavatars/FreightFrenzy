package org.firstinspires.ftc.teamcode.OpenCV.AprilTag;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpenCV.Vision;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;

@Config
@TeleOp(name = "April Tag Test")
public class AprilTagTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Vision detector = new Vision(this, Vision.Pipeline.AprilTag, 0.166);
        detector.start();

        waitForStart();

        while (opModeIsActive()) {
            detector.getAprilTagPipe().runAprilTag();



            telemetry.addData("X: ", detector.getAprilTagPipe().getLocation()[0]);
            telemetry.addData("Y: ", detector.getAprilTagPipe().getLocation()[1]);
            telemetry.addData("Z: ", detector.getAprilTagPipe().getLocation()[2]);
            telemetry.addData("Yaw: ", detector.getAprilTagPipe().getLocation()[3]);
            telemetry.update();

            addPacket("X: ", detector.getAprilTagPipe().getLocation()[0]);
            addPacket("Y: ", detector.getAprilTagPipe().getLocation()[1]);
            addPacket("Z: ", detector.getAprilTagPipe().getLocation()[2]);
            addPacket("Yaw: ", detector.getAprilTagPipe().getLocation()[3]);
        }
    }
}