package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp
public class ODStest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpticalDistanceSensor intakeSensor = hardwareMap.get(OpticalDistanceSensor.class, "intakeSensor");

        waitForStart();

        while (opModeIsActive()) {
            addPacket("Raw",    intakeSensor.getRawLightDetected());
            addPacket("Normal", intakeSensor.getLightDetected());
            sendPacket();

            telemetry.addData("Raw",    intakeSensor.getRawLightDetected());
            telemetry.addData("Normal", intakeSensor.getLightDetected());
            telemetry.update();
        }
    }
}
