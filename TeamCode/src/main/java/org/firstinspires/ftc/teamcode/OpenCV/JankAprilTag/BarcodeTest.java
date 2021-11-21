package org.firstinspires.ftc.teamcode.OpenCV.JankAprilTag;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.*;

@Config
@TeleOp(name = "Jank April Tag Pipeline Test")
public class BarcodeTest extends LinearOpMode {
    public static boolean isRed = false;

    private BarcodeDetector detector;

    @Override
    public void runOpMode() {
        detector = new BarcodeDetector(this, isRed);

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
