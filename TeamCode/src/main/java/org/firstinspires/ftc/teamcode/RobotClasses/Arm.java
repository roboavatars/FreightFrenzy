package org.firstinspires.ftc.teamcode.RobotClasses;

import static java.lang.Math.PI;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
@Config
public class Arm {
    private DcMotorEx armMotor;
    private Servo depositServo;

    private double lastServoPos = 0;

    public double ARM_TICKS_PER_RADIAN = 540 / PI;
    public static int DEPOSIT_ARM_ERROR_THRESHOLD = 20;

    public int targetArmPos = 0;

    private double initialArmAngle = -PI / 6;

    // Arm PD
    double armErrorChange = 0, armError = 0;
    public static double pArmUp = 0.004;
    public static double dArmUp = 0.04;
    public static double fGravityUp = 0.1;

    public static double pArmDown = 0.002;
    public static double dArmDown = 0.005;
    public static double fGravityDown = 0;

    public static double pArm = pArmUp;
    public static double dArm = dArmUp;
    public static double fGravity = fGravityUp;

    public static double armMaxPower = 1;

    public boolean depositing = false;

    public double lastArmPos = 0;

    private boolean isAuto;
    public boolean preload = false;

    public boolean reset = false;

    public int initialArmPos;

    public Arm(LinearOpMode op, boolean isAuto) {
        this(op, isAuto, 0, 0);
    }

    public Arm(LinearOpMode op, boolean isAuto, int initialArmPos, int initialSlidesPos) {
        this.isAuto = isAuto;

        // Deposit Servo
        depositServo = op.hardwareMap.get(Servo.class, "depositServo");
        if (isAuto) {
            hold();
        } else {
            open();
        }

        // Arm Motor
        armMotor = op.hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setDirection(DcMotorEx.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setHome();

        this.initialArmPos = initialArmPos;

        op.telemetry.addData("Status", "Deposit Initialized");
    }

    // Slides + Arm
    public void setHome() {
        depositing = false;
        setArmPIDCoefficients(pArmDown, dArmDown, fGravityDown);
        setArmTarget(Constants.ARM_HOME_POS);
    }

    public void setHigh() {
        targetArmPos = Constants.ARM_HIGH_POS;
        setArmPIDCoefficients(pArmUp, dArmUp, fGravityUp);
    }

    public void setNeutral() {
        targetArmPos = Constants.ARM_NEUTRAL_POS;
        setArmPIDCoefficients(pArmUp, dArmUp, fGravityUp);
    }

    public void update() {
        double targetTicks = targetArmPos;
        double currentTicks = getArmPosition();
        armErrorChange = targetTicks - currentTicks - armError;
        armError = targetTicks - currentTicks;

        double fArm = fGravity * Math.cos(getArmAngle());
        armMotor.setPower(Math.min(pArm * armError + dArm * armErrorChange + fArm, armMaxPower));

        Log.w("deposit-log", "arm set to: " + targetArmPos + ", current position: " + getArmPosition() + ", power " + (pArm * armError + dArm * armErrorChange + fArm));
    }

    public void setArmTarget(int targetPos) {
        targetArmPos = Math.min(Math.max(targetPos, 0), Constants.ARM_HIGH_POS);
    }

    public int getArmPosition() {
        return armMotor.getCurrentPosition() + initialArmPos;
    }

    public double getArmVelocity() {
        return armMotor.getVelocity();
    }

    public boolean armAtPos() {
        return Math.abs(getArmPosition() - targetArmPos) < DEPOSIT_ARM_ERROR_THRESHOLD;
    }

    public boolean clearNeutralPipe() {
        return getArmPosition() < Constants.ARM_ROTATE_TURRET_THRESHOLD;
    }

    public boolean armHome() {
        return Math.abs(getArmPosition() - Constants.ARM_HOME_POS) < DEPOSIT_ARM_ERROR_THRESHOLD;
    }

    public double getArmAngle() {
        return Math.min(getArmPosition(), 800) / ARM_TICKS_PER_RADIAN + initialArmAngle;
    }

    public boolean armAtPosPercent(double percent) {
        return getArmPosition() > targetArmPos * percent;
    }

    public double getArmError() {
        return Math.abs(getArmPosition() - targetArmPos);
    }

    public void setArmPIDCoefficients(double p, double d, double f) {
        pArm = p;
        dArm = d;
        fGravity = f;
    }

    // Deposit
    private void setServoPosition(double pos) {
        if (pos != lastServoPos) {
            depositServo.setPosition(pos);
            lastServoPos = pos;
        }
    }

    public void open() {
        setServoPosition(Constants.DEPOSIT_OPEN_POS);
    }

    public void hold() {
        setServoPosition(Constants.DEPOSIT_HOLD_POS);
    }

    public boolean armAtDeposit() {
        return getArmPosition() > Constants.ARM_HIGH_POS - DEPOSIT_ARM_ERROR_THRESHOLD;
    }

    public boolean armSlidesHome() {
        return getArmPosition() < DEPOSIT_ARM_ERROR_THRESHOLD;
    }

    public void resetArmEncoder() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
