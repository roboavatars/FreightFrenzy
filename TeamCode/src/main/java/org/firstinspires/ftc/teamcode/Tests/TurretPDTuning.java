package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Pathing.Ramsete.PDController;
import org.firstinspires.ftc.teamcode.RobotClasses.Constants;

@TeleOp
@Config
public class TurretPDTuning extends LinearOpMode {
    public static double Kp = 2.25;
    public static double Kd = 5.5;

    public static double initialTheta = 0;
    public static double targetTheta;

    private double turretTheta;
    private double turretError;
    private double turretErrorChange;

    public static double TURRET_MIN_THETA = -PI/2;
    public static double TURRET_MAX_THETA = PI/2;
    public static double TICKS_PER_RADIAN = 103.6 * 20 / (2*PI);

    @Override
    public void runOpMode() {
        DcMotorEx turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            double clippedTargetTheta = Math.min(Math.max(targetTheta, TURRET_MIN_THETA), TURRET_MAX_THETA);
            turretTheta = turretMotor.getCurrentPosition() / TICKS_PER_RADIAN + initialTheta;
            turretErrorChange = clippedTargetTheta - turretTheta - turretError;
            turretError = clippedTargetTheta - turretTheta;

            double power = Math.max(Math.min((Kp * turretError + Kd * turretErrorChange), Constants.MAX_TURRET_POWER), -Constants.MAX_TURRET_POWER);
            turretMotor.setPower(power);

            addPacket("theta", turretTheta);
            addPacket("error", turretError);
            addPacket("power", power);
            sendPacket();

        }
    }
}
