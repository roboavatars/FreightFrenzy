package org.firstinspires.ftc.teamcode.OpenCV.JankAprilTag;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.*;

@TeleOp(name = "Jank April Tag Pipeline Test")
public class JankAprilTagTest extends LinearOpMode {
    private JankAprilTagDetector detector;

    @Override
    public void runOpMode() {
        detector = new JankAprilTagDetector(this);

        waitForStart();
        detector.start();

        while (opModeIsActive()) {
            addPacket("FPS", detector.getFPS());
            addPacket("Result", detector.getResult().name());
            sendPacket();
        }

        detector.stop();
    }
}
