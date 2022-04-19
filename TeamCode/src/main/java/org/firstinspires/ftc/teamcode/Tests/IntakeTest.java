package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

//Config in Constants.java
@TeleOp(name = "Intake Test")
@Config
public class IntakeTest extends LinearOpMode {
    public static double intakePower = 1;
    public static double reversePower = -1;
    public static double upPos = 1;
    public static double downPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        Servo flipServo = hardwareMap.get(Servo.class, "intakeServo");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.right_trigger > .1) {
                intakeMotor.setPower(intakePower);
                flipServo.setPosition(downPos);
            } else {
                flipServo.setPosition(upPos);
                if (gamepad1.left_trigger > .1) intakeMotor.setPower(reversePower);
                else intakeMotor.setPower(0);
            }
        }
    }
}
