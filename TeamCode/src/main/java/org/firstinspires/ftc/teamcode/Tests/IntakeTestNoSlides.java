package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Deposit;

@TeleOp
@Config
public class IntakeTestNoSlides extends LinearOpMode {
    private DcMotorEx armMotor;
    private Servo intakeServo;
    private DcMotorEx intake;

    public static double servoPos;
    public static double intakePower;

    private double error;
    private double errorChange;
    private double power;

    @Override
    public void runOpMode() {
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        waitForStart();

        while (opModeIsActive()) {
            double currentTicks = armMotor.getCurrentPosition();
            errorChange = Constants.DEPOSIT_ARM_HOME_TICKS - currentTicks - error;
            error = Constants.DEPOSIT_ARM_HOME_TICKS - currentTicks;

            armMotor.setPower(Deposit.pArm * error + Deposit.dArm * errorChange);

            intake.setPower(intakePower);
            intakeServo.setPosition(servoPos);
        }
    }
}
