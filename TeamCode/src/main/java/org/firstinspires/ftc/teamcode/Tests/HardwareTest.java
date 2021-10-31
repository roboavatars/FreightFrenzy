package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Deposit;


@TeleOp
@Config
public class HardwareTest extends LinearOpMode {
    public static String motorName1 = "intake";
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
        DcMotor motor1 = hardwareMap.get(DcMotor.class, motorName1);
//        DcMotor motor3 = hardwareMap.get(DcMotor.class, motorName3);
        Deposit motor3 = new Deposit(this);
//        Deposit motor3 = hardwareMap.get(DcMotor.class, motorName3);
        Servo servo1 = hardwareMap.get(Servo.class, servoName1);

        waitForStart();

        while(opModeIsActive()) {
//            if (home) {
//                servo1.setPosition(servo3Home);
//            } else {
//                servo1.setPosition(servo3Out);
//            }

            telemetry.addData("value", motor3.getPosition());
            telemetry.update();

            if (gamepad1.a) {
                motor3.moveSlides(motor3Power, Deposit.deposit_height.HOME);
            }

            if (gamepad1.b) {
                motor3.moveSlides(motor3Power, Deposit.deposit_height.MID);
            }

            if (gamepad1.x) {
                motor3.moveSlides(motor3Power, Deposit.deposit_height.TOP);
            }

            if (gamepad1.y) {
                motor3.moveSlides(motor3Power, Deposit.deposit_height.CAP);
            }

            if (gamepad1.right_bumper) {
                servo1.setPosition(servo1Home);
            }

            if (gamepad1.left_bumper) {
                servo1.setPosition(servo1Out);
            }

            if (gamepad1.right_trigger>0.1) {
                motor1.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger>0.1) {
                motor1.setPower(-gamepad1.left_trigger);
            } else {
                motor1.setPower(0);
            }

//            motor1.setPower(motor1Power);
//            motor2.setPower(motor2Power);
//            motor3.setPower(motor3Power);

//            if (updateMotor) {
//                motor3.setTargetPosition((int) motor3Tick);
//            }

        }
    }
}