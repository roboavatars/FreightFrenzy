package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class ArmTest extends LinearOpMode {
    public static double servo1pos = 0;
    public static double offset = 0;

    @Override
    public void runOpMode() {
        Servo arm1 = hardwareMap.get(Servo.class, "arm1");
        Servo arm2 = hardwareMap.get(Servo.class, "arm2");

        waitForStart();

        while (opModeIsActive()) {
            arm1.setPosition(servo1pos);
            arm2.setPosition(1-servo1pos+offset);
        }
    }

}
