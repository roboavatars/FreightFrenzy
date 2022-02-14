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

    // Turret PD
    public static double pTurret = 2.25;
    public static double dTurret = 5.5;
    public double initialTheta;

    public static double fwTurret = -0.4;   // TODO: remove usages and delete
    public static double fmoiTurret = 0;    // TODO: remove usages and delete

    private double turretTargetTheta;
    public double turretTheta;
    private double turretError = 0;
    private double turretErrorChange;

    private boolean home = true;

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

    public void update(boolean slidesHome) {

        //Move Turret Home Only If Set To Home And Slides Are Home
        if (home && slidesHome){
            turretTargetTheta = Math.min(Math.max(initialTheta, TURRET_MIN_THETA), TURRET_MAX_THETA);;
        }

        turretTheta = getTurretTheta();
        turretErrorChange = turretTargetTheta - turretTheta - turretError;
        turretError = turretTargetTheta - turretTheta;

        setTurretPower(pTurret * turretError + dTurret * turretErrorChange);
    }

    // Set Controls
    public void setDepositing(double theta) { // TODO: make private method
        home = false;
        turretTargetTheta = Math.min(Math.max(theta, TURRET_MIN_THETA), TURRET_MAX_THETA);;
    }

    public void setHome() {
        home = true;
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

    public void setTurretThetaFF(double theta, double ff) {        // TODO: remove usages and delete
        double clippedTargetTheta = Math.min(Math.max(theta, TURRET_MIN_THETA), TURRET_MAX_THETA);
        turretTheta = getTurretTheta();
        turretErrorChange = clippedTargetTheta - turretTheta - turretError;
        turretError = clippedTargetTheta - turretTheta;

        setTurretPower(pTurret * turretError + dTurret * turretErrorChange + fwTurret * ff/* + fmoiTurret * getSlidesDistInches() * getSlidesDistInches()*/);
    }
}
