package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;

@TeleOp(name = "Arm Test")
@SuppressWarnings("FieldCanBeLocal")
@Config
public class ArmTest extends LinearOpMode {
    private Servo armServo1;
    private Servo armServo2;

    public static int pos = 0;

    double armPos;

    @Override
    public void runOpMode() {
        armServo1 = hardwareMap.get(Servo.class, "arm1");
        armServo2 = hardwareMap.get(Servo.class, "arm2");

        waitForStart();
        while (opModeIsActive()) {
            if (pos == 0){
                armPos = Constants.DEPOSIT_ARM_HOME;
            } else if (pos == 1){
                armPos = Constants.DEPOSIT_ARM_TOP_GOAL;
            } else if (pos == 2){
                armPos = Constants.DEPOSIT_ARM_MID_GOAL;
            } else if (pos == 3){
                armPos = Constants.DEPOSIT_ARM_TOP_GOAL;
            } else {
                armPos = Constants.DEPOSIT_ARM_CAP;
            }
            armServo1.setPosition(Math.min(Math.max(armPos, 0),1));
            armServo2.setPosition(Math.min(Math.max(1 - armPos + Constants.DEPOSIT_ARM_SERVO_OFFSET, 0),1));
        }
    }
}