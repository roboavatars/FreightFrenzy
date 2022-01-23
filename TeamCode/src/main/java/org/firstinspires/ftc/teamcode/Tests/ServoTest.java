package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class ServoTest extends LinearOpMode {
    public static String servoName1 = "intakeSlides";

    public static double pos = 0;

    @Override
    public void runOpMode() {
        Servo servo1 = hardwareMap.get(Servo.class, servoName1);

        waitForStart();

        while (opModeIsActive()) {
            servo1.setPosition(pos);
        }
    }
}