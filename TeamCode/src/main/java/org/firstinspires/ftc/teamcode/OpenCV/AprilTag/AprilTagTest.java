package org.firstinspires.ftc.teamcode.OpenCV.AprilTag;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "April Tag Test")
@Disabled
public class AprilTagTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        AprilTagDetector detector = new AprilTagDetector(this);
        detector.start();

        waitForStart();

        while (opModeIsActive()) {
            detector.runAprilTag();

            telemetry.addData("X: ", detector.getLocation()[0]);
            telemetry.addData("Y: ", detector.getLocation()[1]);
            telemetry.addData("Z: ", detector.getLocation()[2]);
            telemetry.addData("Yaw: ", detector.getLocation()[3]);
            telemetry.update();

            addPacket("X: ", detector.getLocation()[0]);
            addPacket("Y: ", detector.getLocation()[1]);
            addPacket("Z: ", detector.getLocation()[2]);
            addPacket("Yaw: ", detector.getLocation()[3]);
            sendPacket();
        }
    }
}
