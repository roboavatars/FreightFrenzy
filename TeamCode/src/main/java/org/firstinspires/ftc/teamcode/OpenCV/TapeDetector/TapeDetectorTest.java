package org.firstinspires.ftc.teamcode.OpenCV.TapeDetector;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Tape Detector Pipeline Test")
public class TapeDetectorTest extends LinearOpMode {
    private TapeDetector detector;
    private double[] result;

    @Override
    public void runOpMode() {
        detector = new TapeDetector(this);

        waitForStart();
        detector.start();

        while (opModeIsActive()) {
            result = detector.getResult();
            addPacket("FPS", detector.getFPS());
            addPacket("Result", "(" + result[0] + ", " + result[1] + ")");
            sendPacket();
        }

        detector.stop();
    }
}
