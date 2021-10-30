package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotClasses.Deposit;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawDrivetrain;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawField;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;


@TeleOp
@Config
public class HardwareTest extends LinearOpMode {
    public static String motorName1 = "intake";
//    public static String motorName2 = "transfer";
    public static String motorName3 = "depositor";
    public static String servoName1 = "leftClamp";
    public static String servoName2 = "rightClamp";
    public static String servoName3 = "depositServo";

    public static double motor1Power = 0;
//    public static double motor2Power = 0;
    public static double motor3Power = 0.5;
    public static double motor3Tick = 0;
    public static double servo1Home = 0.95;
    public static double servo1Out = 0.4;
    public static double servo2Home = 0.05;
    public static double servo2Out = 0.6;
    public static double servo3Home = 0;
    public static double servo3Out = 1;

    public static boolean updateMotor = false;
    public static boolean home = true;

    @Override
    public void runOpMode() {
        DcMotor motor1 = hardwareMap.get(DcMotor.class, motorName1);
//        DcMotor motor2 = hardwareMap.get(DcMotor.class, motorName2);
//        DcMotor motor3 = hardwareMap.get(DcMotor.class, motorName3);
        Deposit motor3 = new Deposit(this);
//        Deposit motor3 = hardwareMap.get(DcMotor.class, motorName3);
        Servo servo1 = hardwareMap.get(Servo.class, servoName1);
        Servo servo2 = hardwareMap.get(Servo.class, servoName2);
        Servo servo3 = hardwareMap.get(Servo.class, servoName3);

        waitForStart();

        while(opModeIsActive()) {
            if (home) {
                servo1.setPosition(servo1Home);
                servo2.setPosition(servo2Home);
                servo3.setPosition(servo3Home);
            } else {
                servo1.setPosition(servo1Out);
                servo2.setPosition(servo2Out);
                servo3.setPosition(servo3Out);
            }

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

//            motor1.setPower(motor1Power);
//            motor2.setPower(motor2Power);
//            motor3.setPower(motor3Power);

//            if (updateMotor) {
//                motor3.setTargetPosition((int) motor3Tick);
//            }

        }
    }
}