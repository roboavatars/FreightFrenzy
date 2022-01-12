package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;

@TeleOp
@Config
public class MotorPdTest extends LinearOpMode {
    public static boolean move = false;
    public static int targetTicks = 0;
    public static String motorName = "motor";

    public static double Kp = 2.25;
    public static double Kd = 5.5;

    private double error;
    private double errorChange;
    private double power;

    @Override
    public void runOpMode() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            if (move) {
                double currentTicks = motor.getCurrentPosition();
                errorChange = targetTicks - currentTicks - error;
                error = targetTicks - currentTicks;

                power = Kp * error + Kd * errorChange;
            } else {
                power = 0;
            }
            motor.setPower(power);

            addPacket("ticks", motor.getCurrentPosition());
            addPacket("error", error);
            addPacket("error change", errorChange);
            addPacket("power", power);
            sendPacket();
        }
    }

}
