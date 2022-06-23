package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp
@Config
public class MotorConfigTest extends LinearOpMode {
    private DcMotorEx motorFrontRight;
    private DcMotorEx motorFrontLeft;
    private DcMotorEx motorBackRight;
    private DcMotorEx motorBackLeft;

    public static boolean motorFrontRightSpinning = false;
    public static boolean motorFrontLeftSpinning = false;
    public static boolean motorBackRightSpinning = false;
    public static boolean motorBackLeftSpinning = false;
    public static double motorPower = 0.25;

    @Override
    public void runOpMode() {
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "motorBackRight");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft");

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            motorFrontLeft.setPower(motorFrontLeftSpinning ? motorPower : 0);
            motorFrontRight.setPower(motorFrontRightSpinning ? motorPower : 0);
            motorBackLeft.setPower(motorBackLeftSpinning ? motorPower : 0);
            motorBackRight.setPower(motorBackRightSpinning ? motorPower : 0);

            addPacket("Front Left", motorFrontLeft.getVelocity());
            addPacket("Front Right", motorFrontRight.getVelocity());
            addPacket("Back Right", motorBackRight.getVelocity());
            addPacket("Back Left", motorBackLeft.getVelocity());
            sendPacket();

        }
    }
}