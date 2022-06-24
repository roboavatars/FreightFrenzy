package org.firstinspires.ftc.teamcode.OpenCV.Barcode;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Barcode Pipeline Test")
public class BarcodeTest extends LinearOpMode {
    public static boolean isRed = false;
    public static boolean isWarehouse = false;

    private BarcodeDetector detector;

    @Override
    public void runOpMode() {
        detector = new BarcodeDetector(this, isRed, isWarehouse);

        waitForStart();
        detector.start();

        while (opModeIsActive()) {
            telemetry.addData("FPS", detector.getFPS());
            telemetry.addData("Result", detector.getResult().name());
            telemetry.update();

            addPacket("FPS", detector.getFPS());
            addPacket("Result", detector.getResult().name());
            sendPacket();
        }

        detector.stop();
    }
}
