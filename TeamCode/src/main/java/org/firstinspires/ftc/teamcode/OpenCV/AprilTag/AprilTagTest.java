package org.firstinspires.ftc.teamcode.OpenCV.AprilTag;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpenCV.Vision;

@TeleOp(name = "April Tag Test")
@Disabled
public class AprilTagTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Vision detector = new Vision(this, Vision.Pipeline.AprilTag);
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
            sendPacket();
        }
    }
}
