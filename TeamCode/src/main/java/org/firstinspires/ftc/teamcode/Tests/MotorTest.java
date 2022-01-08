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
public class MotorTest extends LinearOpMode {
    private DcMotorEx motor;

    public static String motorName = "turret";
    public static double power = 0;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, motorName);

        waitForStart();

        while(opModeIsActive()) {
            motor.setPower(power);

            addPacket("current", motor.getCurrent(CurrentUnit.AMPS));
            addPacket("ticks", motor.getCurrentPosition());
            addPacket("velocity", motor.getVelocity());
            sendPacket();
        }
    }
}