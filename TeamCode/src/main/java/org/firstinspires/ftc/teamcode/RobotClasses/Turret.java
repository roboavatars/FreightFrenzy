package org.firstinspires.ftc.teamcode.RobotClasses;

import static java.lang.Math.PI;

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

    // Turret PD and PDFF
    public static double pTurret = 2.25;
    public static double dTurret = 5.5;
    public static double fwTurret = -0.4;
    public static double fmoiTurret = 0;
    public double initialTheta;

    private double turretTargetTheta;
    public double turretTheta;
    private double turretError = 0;
    private double turretErrorChange;
    private double turretLockTheta;

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

        op.telemetry.addData("Status", "Turret Initialized");
    }

    public void setTurretLockTheta(double lockTheta) {
        turretLockTheta = lockTheta;
    }

    public void turretHome() {
        turretTargetTheta = initialTheta;
    }

    public void update(double robotTheta, double turretFF) {
        turretTargetTheta = (turretLockTheta - robotTheta) % (2 * PI);
        if (turretTargetTheta < 0) {
            turretTargetTheta += 2 * PI;
        }
        // prevents wrap from 0 to 2pi from screwing things up
        // now wrap is from -pi/2 to 3pi/2 (which the turret will never reach)
        if (turretTargetTheta > 3*PI/2) {
            turretTargetTheta -= 2*PI;
        }
        setTurretThetaFF(turretTargetTheta, turretFF);
    }

    // Turret
    public void setTurretThetaFF(double theta, double ff) {
        double clippedTargetTheta = Math.min(Math.max(theta, TURRET_MIN_THETA), TURRET_MAX_THETA);
        turretTheta = getTurretTheta();
        turretErrorChange = clippedTargetTheta - turretTheta - turretError;
        turretError = clippedTargetTheta - turretTheta;

        setTurretPower(pTurret * turretError + dTurret * turretErrorChange + fwTurret * ff/* + fmoiTurret * getSlidesDistInches() * getSlidesDistInches()*/);
    }

    public void setTurretTheta(double theta) { // TODO: make private method
        double clippedTargetTheta = Math.min(Math.max(theta, TURRET_MIN_THETA), TURRET_MAX_THETA);
        turretTheta = getTurretTheta();
        turretErrorChange = clippedTargetTheta - turretTheta - turretError;
        turretError = clippedTargetTheta - turretTheta;

        setTurretPower(pTurret * turretError + dTurret * turretErrorChange);
    }

    public void setTurretPower(double power) {
        turretMotor.setPower(Math.max(Math.min(power, MAX_TURRET_POWER), -MAX_TURRET_POWER));
    }

    public void resetTurret(double resetTheta) {
        initialTheta = resetTheta;
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getTargetTheta() {
        return turretTargetTheta;
    }

    public double getTurretTheta() {
        return turretMotor.getCurrentPosition() / TURRET_TICKS_PER_RADIAN + initialTheta;
    }

    public double getTurretError() {
        return turretError;
    }

    public boolean turretAtPos() {
        return Math.abs(turretError) < TURRET_ERROR_THRESHOLD;
    }
}
