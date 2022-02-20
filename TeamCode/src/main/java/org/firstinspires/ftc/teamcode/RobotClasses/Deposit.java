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
public class Deposit {
    private DcMotorEx slidesMotor;
    private DcMotorEx armMotor;
    private Servo depositServo;

    private double lastServoPos = 0;

    public static double DEPOSIT_SLIDES_TICKS_PER_INCH = 9.142857;
    public int DEPOSIT_SLIDES_MAX_TICKS = (int) (25 * DEPOSIT_SLIDES_TICKS_PER_INCH);
    public int DEPOSIT_SLIDES_ERROR_THRESHOLD = 15;
    public double ARM_TICKS_PER_RADIAN = 1120 / (2*PI);
    public double DEPOSIT_ARM_MAX_POWER = 0.7;
    public static int DEPOSIT_ARM_ERROR_THRESHOLD = 30;

    // Slides PD
    public static double pSlides = 50;

    public int targetSlidesTicks;
    public Robot.DepositTarget target;

    private static final double maxSlidesDistBeforeLoweringArm = 2;

    public boolean home = true;
    public int targetArmPos;

    private double initialArmAngle = -0.646;

    // Arm PD
    double armErrorChange = 0, armError = 0;
    public static double pArmUp   = 0.003;
    public static double dArmUp   = 0.0055;
    public static double pArmDown = 0.0015;
    public static double dArmDown = 0.002;

    public static double pArm = pArmUp;
    public static double dArm = dArmUp;
    public static double fArm = 0;

    public static double fGravity = 0.07;

    public Deposit(LinearOpMode op, boolean isAuto) {
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

        // Slides Motor
        slidesMotor = op.hardwareMap.get(DcMotorEx.class, "depositSlides");
        slidesMotor.setDirection(DcMotorEx.Direction.REVERSE);
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setTargetPosition(0);
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesMotor.setPositionPIDFCoefficients(pSlides);

        setDepositHome();

        op.telemetry.addData("Status", "Deposit Initialized");
    }

    // Slides + Arm
    public void setDepositHome() {
        home = true;
        setArmPIDCoefficients(Deposit.pArmDown, Deposit.dArmDown);
        setArmTarget(Constants.DEPOSIT_ARM_HOME);
        setSlidesTarget(0);
    }

    public void setDepositControls(Robot.DepositTarget target, double slidesDist) {
        home = false;
        this.target = target;
        setArmPIDCoefficients(Deposit.pArmUp, Deposit.dArmUp);

        int targetArmPos = 0;
        if (target == Robot.DepositTarget.allianceLow || target == Robot.DepositTarget.neutral) {
            targetArmPos = Constants.DEPOSIT_ARM_LOW;
        } else if (target == Robot.DepositTarget.allianceMid) {
            targetArmPos = Constants.DEPOSIT_ARM_MID;
        } else if (target == Robot.DepositTarget.allianceHigh || target == Robot.DepositTarget.duck) {
            targetArmPos = Constants.DEPOSIT_ARM_HIGH;
        }

        setArmTarget(targetArmPos);
        setSlidesTarget((int) Math.round(slidesDist * DEPOSIT_SLIDES_TICKS_PER_INCH));
    }

    public void update() {
        // Move Arm
        if (home) {
            if (getSlidesDistInches() < maxSlidesDistBeforeLoweringArm) {
                setArmControls();
            }
            setSlidesControls();
        } else {
            // arm out first if low or mid
            if ((target == Robot.DepositTarget.allianceLow || target == Robot.DepositTarget.allianceMid) && armAtPosPercent(0.75)
                || target == Robot.DepositTarget.allianceHigh) {
                setSlidesControls();
            }
            // midway arm pos if high or duck
            if ((target == Robot.DepositTarget.allianceHigh || target == Robot.DepositTarget.duck) && !slidesAtPos()) {
                setArmControls(Constants.DEPOSIT_ARM_MIDWAY);
            } else {
                setArmControls();
            }
        }
        Log.w("arm-log", targetArmPos+" (" + home + ")");

        // Move Slides
        // Cap Slides Extension Distance When Extending to the Side to Prevent Tipping
        // targetSlidesTicks = (int) Math.min(targetSlidesTicks, Constants.DEPOSIT_SLIDES_SIDE_EXTENSION_LIMIT_IN * Constants.DEPOSIT_SLIDES_TICKS_PER_INCH * Math.abs(1/Math.cos(getTurretTheta())));
    }

    // Arm
    public void setArmControls(int targetArmPos) {
        double targetTicks = (int) Math.min(Math.max(targetArmPos, Constants.DEPOSIT_ARM_HOME), Constants.DEPOSIT_ARM_LOW);
        double currentTicks = getArmPosition();
        armErrorChange = targetTicks - currentTicks - armError;
        armError = targetTicks - currentTicks;

        fArm = fGravity * Math.cos(getArmAngle());
        armMotor.setPower(pArm * armError + dArm * armErrorChange + fArm);
    }

    public void setArmControls() {
        setArmControls(targetArmPos);
    }

    public void setArmTarget(int targetPos){
        targetArmPos = Math.min(Math.max(targetPos, 0), Constants.DEPOSIT_ARM_LOW);
    }

    public double getArmPosition() {
        return armMotor.getCurrentPosition();
    }

    public double getArmVelocity() {
        return armMotor.getVelocity();
    }

    public boolean armAtPos() {
        return Math.abs(getArmPosition() - targetArmPos) < DEPOSIT_ARM_ERROR_THRESHOLD;
    }

    public boolean armHome() {
        return Math.abs(getArmPosition() - Constants.DEPOSIT_ARM_HOME) < DEPOSIT_ARM_ERROR_THRESHOLD;
    }

    public double getArmAngle() {
        return getArmPosition() / ARM_TICKS_PER_RADIAN + initialArmAngle;
    }

    public boolean armAtPosPercent(double percent) {
        return getArmPosition() > targetArmPos * percent;
    }

    public double getArmError() {
        return Math.abs(getArmPosition() - targetArmPos);
    }

    public void setArmPIDCoefficients(double p, double d) {
        pArm = p;
        dArm = d;
    }

    // Slides
    public void setSlidesControls(int targetSlidesPos) {
        slidesMotor.setTargetPosition(targetSlidesPos);
        slidesMotor.setPower(DEPOSIT_ARM_MAX_POWER);
    }

    public void setSlidesControls() {
        setSlidesControls(targetSlidesTicks);
    }

    public void setSlidesTarget(int targetPos){
        targetSlidesTicks = Math.min(Math.max(targetPos, 0), DEPOSIT_SLIDES_MAX_TICKS);
    }

    public double getSlidesPosition() {
        return slidesMotor.getCurrentPosition();
    }

    public double getSlidesDistInches() {
        return getSlidesPosition() / DEPOSIT_SLIDES_TICKS_PER_INCH;
    }

    public boolean slidesAtPos() {
        return Math.abs(getSlidesPosition() - targetSlidesTicks) < DEPOSIT_SLIDES_ERROR_THRESHOLD;
    }

    public boolean slidesAtPosPercent(double percent) {
        return getSlidesPosition() > targetSlidesTicks * percent;
    }

    public boolean slidesHome() {
        return Math.abs(getSlidesPosition()) < DEPOSIT_ARM_ERROR_THRESHOLD;
    }

    public double getSlidesError() {
        return Math.abs(getSlidesPosition() - targetSlidesTicks);
    }

    public void setSlidesPIDCoefficients(double p) {
        slidesMotor.setPositionPIDFCoefficients(p);
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

    public boolean armSlidesAtPose() {
        return armAtPos() && slidesAtPos();
    }

    public boolean armSlidesHome() {
        return armHome() && slidesHome();
    }
}
