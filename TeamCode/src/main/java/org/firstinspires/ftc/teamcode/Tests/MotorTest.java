package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp
@Config
public class MotorTest extends LinearOpMode {
    private DcMotorEx motor;

    public static String motorName = "motorFrontLeft";

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, motorName);

        waitForStart();

        while(opModeIsActive()) {
            motor.setPower(gamepad1.right_stick_y);
        }
    }
}