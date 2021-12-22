package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
@Config
public class ServoTest extends LinearOpMode {
    private Servo servo;

    public static String servoName = "test";
    public static double servoPosition = 0;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, servoName);

        waitForStart();

        while(opModeIsActive()) {
            servo.setPosition(servoPosition);
        }
    }
}