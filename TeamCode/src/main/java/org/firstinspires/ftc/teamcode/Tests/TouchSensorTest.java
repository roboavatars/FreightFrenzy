package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
@Config
public class TouchSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        TouchSensor touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        waitForStart();

        while (opModeIsActive()) {
            addPacket("is pressed", touchSensor.isPressed());
            addPacket("sensor value", touchSensor.getValue());
            sendPacket();

            telemetry.addData("is pressed", touchSensor.isPressed());
            telemetry.addData("sensor value", touchSensor.getValue());
            telemetry.update();
        }
    }
}
