package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp
@Config
public class MotorRunToTest extends LinearOpMode {
    private DcMotorEx motor;

    public static String motorName = "motor";
    public static double power = 0;
    public static boolean runToPos = false;
    public static int ticks = 0;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);

        waitForStart();

        while(opModeIsActive()) {
            if (runToPos){
                motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                motor.setPower(power);
                motor.setTargetPosition(ticks);
            } else {
                motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                motor.setPower(power);
            }

            addPacket("current", motor.getCurrent(CurrentUnit.AMPS));
            addPacket("ticks", motor.getCurrentPosition());
            addPacket("velocity", motor.getVelocity());
            sendPacket();
        }
    }
}