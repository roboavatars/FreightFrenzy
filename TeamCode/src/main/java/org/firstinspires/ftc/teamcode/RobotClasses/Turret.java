package org.firstinspires.ftc.teamcode.RobotClasses;

import static java.lang.Math.PI;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@SuppressWarnings("FieldCanBeLocal")
@Config
public class Turret {
    private DcMotorEx turretMotor;

    public double MAX_TURRET_POWER = 1;
    public double TURRET_MIN_THETA = 0;
    public double TURRET_MAX_THETA = PI;
    public double TURRET_TICKS_PER_RADIAN = 103.6 * 20 / (2*PI);
    public static double TURRET_Y_OFFSET = 2.06066;
    public double TURRET_ERROR_THRESHOLD = PI/40;
    public double TURRET_NEUTRAL_THRESHOLD = PI/40;

    // Turret PD
    public static double pTurret = 2.25;
    public static double dTurret = 5.5;
    public double initialTheta;
    private boolean isAuto;

    private double turretTargetTheta;
    public double turretTheta;
    private double turretError = 0;
    private double turretErrorChange;

    public Turret(LinearOpMode op, boolean isAuto) {
        this(op, isAuto, PI/2);
    }

    public Turret(LinearOpMode op, boolean isAuto, double initialRobotTheta) {

        // Turret Motor
        turretMotor = op.hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set Initial Turret Theta
        initialTheta = initialRobotTheta;
        setHome();

        this.isAuto = isAuto;

        op.telemetry.addData("Status", "Turret Initialized");
    }

    public void update() {
        turretTheta = getTheta();
        turretErrorChange = turretTargetTheta - turretTheta - turretError;
        turretError = turretTargetTheta - turretTheta;

        setPower(pTurret * turretError + dTurret * turretErrorChange);

        Log.w("turret-log", getTheta() + "");
    }

    // Set Controls
    public void setNeutral() {
        setTheta(Constants.TURRET_NETRUAL_THETA);
    }
    public void setTheta(double theta) {
        turretTargetTheta = Math.min(Math.max(theta, TURRET_MIN_THETA), TURRET_MAX_THETA);
    }

    public void setHome() {
        setTheta(PI/2);
    }

    public void setPower(double power) {
        turretMotor.setPower(Math.max(Math.min(power, MAX_TURRET_POWER), -MAX_TURRET_POWER));
    }

    public void reset(double resetTheta) {
        initialTheta = resetTheta;
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getTargetTheta() {
        return turretTargetTheta;
    }

    public double getTheta() {
        return turretMotor.getCurrentPosition() / TURRET_TICKS_PER_RADIAN + initialTheta;
    }

    public double getError() {
        return turretError;
    }

    public boolean isHome() {
        return Math.abs(getTheta() - initialTheta) < TURRET_ERROR_THRESHOLD;
    }

    public boolean isAtNeutral() {
        return getTheta() > Constants.TURRET_NETRUAL_THETA - TURRET_NEUTRAL_THRESHOLD;
    }
}