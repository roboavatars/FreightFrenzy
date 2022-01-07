package org.firstinspires.ftc.teamcode.Teleop;

import static java.lang.Math.PI;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Debug.Logger;
import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Deposit;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.ArrayList;
import java.util.Arrays;

@TeleOp(name = "Arm Test")
@SuppressWarnings("FieldCanBeLocal")
@Config
public class ArmTest extends LinearOpMode {
    private Servo armServo1;
    private Servo armServo2;

    public static int pos = 0;

    public static double arm1_home = 0.5;
    public static double arm1_top = 0.5;
    public static double arm1_mid = 0.5;
    public static double arm1_low = 0.5;


    @Override
    public void runOpMode() {
        armServo1 = hardwareMap.get(Servo.class, "arm1");
        armServo2 = hardwareMap.get(Servo.class, "arm2");

        waitForStart();
        while (opModeIsActive()) {
            double arm2_home = 1.05 - arm1_home;
            double arm2_top = 1.05 - arm1_top;
            double arm2_mid = 1.05 - arm1_mid;
            double arm2_low = 1.05 - arm1_low;

            if (pos == 0){
                armServo1.setPosition(arm1_home);
                armServo2.setPosition(arm2_home);
            } else if (pos == 1){
                armServo1.setPosition(arm1_top);
                armServo2.setPosition(arm2_top);
            } else if (pos == 2){
                armServo1.setPosition(arm1_mid);
                armServo2.setPosition(arm2_mid);
            } else if (pos == 3){
                armServo1.setPosition(arm1_low);
                armServo2.setPosition(arm2_low);
            }
        }
    }
}