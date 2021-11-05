package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Deposit;


@TeleOp
@Config
public class HardwareTest extends LinearOpMode {
    public static String motorName1 = "depositor";
    public static String motorName3 = "depositor";
    public static String servoName1 = "depositServo";

    public static double motor1Power = 0;
    public static double motor3Power = 1.0;
    public static double motor3Tick = 0;
    public static double servo1Home = 0;
    public static double servo1Out = 1;

    public static boolean updateMotor = false;
    public static boolean home = true;

    @Override
    public void runOpMode() {
        DcMotor motor1 = hardwareMap.get(DcMotorEx.class, motorName1);
//        DcMotor motor3 = hardwareMap.get(DcMotor.class, motorName3);
        Deposit motor3 = new Deposit(this);
//        Deposit motor3 = hardwareMap.get(DcMotor.class, motorName3);
        Servo servo1 = hardwareMap.get(Servo.class, servoName1);

        waitForStart();

        while(opModeIsActive()) {
            motor1.setPower(gamepad1.right_stick_y);
            telemetry.addData("encoder ticks", motor1.getCurrentPosition());
            telemetry.update();

//            motor1.setPower(motor1Power);
//            motor2.setPower(motor2Power);
//            motor3.setPower(motor3Power);

//            if (updateMotor) {
//                motor3.setTargetPosition((int) motor3Tick);
//            }

        }
    }
}