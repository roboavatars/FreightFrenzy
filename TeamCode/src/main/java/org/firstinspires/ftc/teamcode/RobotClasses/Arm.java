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
    private Servo limitServo;
    private Servo capServo;

    private double lastServoPos = 0; // deposit servo
    private double lastServoPos2 = 0; // limit servo

    public double ARM_TICKS_PER_RADIAN = 540 / PI;
    public static int DEPOSIT_ARM_HOME_THRESHOLD = 50;
    public static int DEPOSIT_ARM_DEPOSIT_THRESHOLD = 30;

    public int targetArmPos = 0;

    private double initialArmAngle = -PI / 6;

    // Arm PD
    double armErrorChange = 0, armError = 0;
    public static double pArmUp = 0.004;
    public static double dArmUp = 0.045;
    public static double fGravityUp = 0.1;

    public static double pArmDown = 0.002;
    public static double dArmDown = 0.05;
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
        limitServo = op.hardwareMap.get(Servo.class, "limitServo");

        if (isAuto) {
            hold();
        } else {
            open();
        }

        limitArm(true);

        // Arm Motor
        armMotor = op.hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setDirection(DcMotorEx.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        capServo = op.hardwareMap.get(Servo.class, "capServo");
        cap(Constants.SERVO_CAP_HOME);

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
        depositing = true;
        targetArmPos = Constants.ARM_HIGH_POS;
        setArmPIDCoefficients(pArmUp, dArmUp, fGravityUp);
    }

//    public void setNeutral() {
//        depositing = true;
//        targetArmPos = Constants.ARM_NEUTRAL_POS;
//        setArmPIDCoefficients(pArmUp, dArmUp, fGravityUp);
//    }
//
//    public void setCapping(int armPos, double pArm, double dArm, double fArm) {
//        setArmTarget(armPos);
//        setArmPIDCoefficients(pArm, dArm, fArm);
//    }
//
//    public void setCapping(int armPos) {
//        setCapping(armPos, pArmUp, dArmUp, fGravityUp);
//    }

    public void update() {
        double targetTicks = targetArmPos;
        double currentTicks = getArmPosition();
        armErrorChange = targetTicks - currentTicks - armError;
        armError = targetTicks - currentTicks;

        double fArm = fGravity * Math.cos(getArmAngle());
        if (!depositing || !armAtDeposit())
                armMotor.setPower(Math.min(pArm * armError + dArm * armErrorChange + fArm, armMaxPower));
        else armMotor.setPower(Constants.ARM_HOLD_DEPOSIT_POWER);

        Log.w("deposit-log", "arm set to: " + targetArmPos + ", current position: " + getArmPosition() + ", power " + (pArm * armError + dArm * armErrorChange + fArm));
    }

    public void setArmTarget(int targetPos) {
        targetArmPos = Math.max(targetPos, 0);
    }

    public int getArmPosition() {
        return armMotor.getCurrentPosition() + initialArmPos;
    }

    public double getArmVelocity() {
        return armMotor.getVelocity();
    }

    public boolean clearNeutralPipe() {
        return getArmPosition() < Constants.ARM_ROTATE_TURRET_THRESHOLD;
    }

    public boolean armHome() {
        return Math.abs(getArmPosition() - Constants.ARM_HOME_POS) < DEPOSIT_ARM_HOME_THRESHOLD;
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
    private void setDepositServoPosition(double pos) {
        if (pos != lastServoPos) {
            depositServo.setPosition(pos);
            lastServoPos = pos;
        }
    }

    private void setLimitServoPosition(double pos) {
        if (pos != lastServoPos) {
            limitServo.setPosition(pos);
            lastServoPos2 = pos;
        }
    }

    public void open() {
        setDepositServoPosition(Constants.DEPOSIT_OPEN_POS);
    }

    public void hold() {
        setDepositServoPosition(Constants.DEPOSIT_HOLD_POS);
    }

    public void release() {
        setDepositServoPosition(Constants.DEPOSIT_RELEASE_POS);
    }
    public void servoCapPos() {
        setDepositServoPosition(Constants.DEPOSIT_CAP_POS);
    }

    public void limitArm(boolean limit) {
        if (limit) {
            setLimitServoPosition(Constants.LIMIT_OPEN);
        }

        if (!limit) {
            setLimitServoPosition(Constants.LIMIT_CLOSE);
        }
    }

    public boolean armAtDeposit() {
        return getArmPosition() > Constants.ARM_HIGH_POS - DEPOSIT_ARM_DEPOSIT_THRESHOLD;
    }

    public boolean armSlidesHome() {
        return armHome();
    }

    public void cap(double pos) {
        capServo.setPosition(pos);
    }

    public void resetArmEncoder() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
