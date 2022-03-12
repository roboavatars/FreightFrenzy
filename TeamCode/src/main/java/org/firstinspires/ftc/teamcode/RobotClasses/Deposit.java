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

    public double DEPOSIT_SLIDES_TICKS_PER_INCH = 9.142857;
    public int DEPOSIT_SLIDES_MAX_TICKS = (int) Math.round(25 * DEPOSIT_SLIDES_TICKS_PER_INCH);
    public static int DEPOSIT_SLIDES_ERROR_THRESHOLD = 15;
    public double ARM_TICKS_PER_RADIAN = 540 / PI;
    public static double DEPOSIT_SLIDES_MAX_POWER = 0.7;
    public static int DEPOSIT_ARM_ERROR_THRESHOLD = 20;
    public static double maxSlidesDistBeforeLoweringArm = 8;

    // Slides PD
    public static double pSlides = 50;

    public int targetSlidesTicks;
    public Robot.DepositTarget target;
    private double slidesDist;

    public int targetArmPos;
    private int targetArmPosNoOffset = 0;

    private double initialArmAngle = -PI/6;
    public int armOffset = 0;
    public int slidesOffset = 0;

    // Arm PD
    double armErrorChange = 0, armError = 0;
    public static double pArmUp = 0.003;
    public static double dArmUp = 0.0055;
    public static double fGravityUp = 0.1;

    public static double pArmDown = 0.0015;
    public static double dArmDown = 0.002;
    public static double fGravityDown = 0.3;

    public static double pArmUpPreload = 0.003;
    public static double dArmUpPreload = 0.035;

    public static double pArm = pArmUp;
    public static double dArm = dArmUp;
    public static double fGravity = fGravityUp;

    public boolean depositing = false;

    public double lastArmPos = 0;
    public double lastSlidesPos = 0;

    private boolean isAuto;
    public boolean preload = false;

    private int initialArmPos;
    private int initialSlidesPos;

    public Deposit(LinearOpMode op, boolean isAuto) {
        this(op, isAuto, 0, 0);
    }
    public Deposit(LinearOpMode op, boolean isAuto, int initialArmPos, int initialSlidesPos) {
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

        // Slides Motor
        slidesMotor = op.hardwareMap.get(DcMotorEx.class, "depositSlides");
        slidesMotor.setDirection(DcMotorEx.Direction.REVERSE);
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setTargetPosition(0);
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesMotor.setPositionPIDFCoefficients(pSlides);

        setDepositHome();

        this.initialArmPos = initialArmPos;
        this.initialSlidesPos = initialSlidesPos;

        op.telemetry.addData("Status", "Deposit Initialized");
    }

    // Slides + Arm
    public void setDepositHome() {
        depositing = false;
        setArmPIDCoefficients(pArmDown, dArmDown, fGravityDown);
        setArmTarget(Constants.DEPOSIT_ARM_HOME - armOffset);
        setSlidesInches(Constants.SLIDES_DISTANCE_HOME - slidesOffset);
    }

    public void setDepositControls(Robot.DepositTarget target, double slidesDist) {
        depositing = true;
        this.target = target;
        this.slidesDist = slidesDist;

        if (preload) setArmPIDCoefficients(pArmUpPreload, dArmUpPreload, fGravityUp);
        else setArmPIDCoefficients(pArmUp, dArmUp, fGravityUp);

        if (target == Robot.DepositTarget.allianceLow || target == Robot.DepositTarget.neutral) {
            targetArmPosNoOffset = Constants.DEPOSIT_ARM_LOW;
        } else if (target == Robot.DepositTarget.allianceMid) {
            targetArmPosNoOffset = Constants.DEPOSIT_ARM_MID;
        } else if (target == Robot.DepositTarget.allianceHigh || target == Robot.DepositTarget.duck) {
            targetArmPosNoOffset = Constants.DEPOSIT_ARM_HIGH;
        }

        setArmTarget(targetArmPosNoOffset);
        setSlidesInches(slidesDist);
    }

    public void update(){
        update(false, true);
    }

    public void update(boolean intakeTransfer, boolean turretHome) {
        if (!depositing) {
            if (target == Robot.DepositTarget.allianceHigh && getSlidesDistInches() >= maxSlidesDistBeforeLoweringArm) {
                setArmPIDCoefficients(pArmDown, dArmDown, fGravityDown);
                setArmControls(Constants.DEPOSIT_ARM_MIDWAY);
            } else if (getSlidesDistInches() < maxSlidesDistBeforeLoweringArm && !armHome() && turretHome) {
                setArmPIDCoefficients(pArmDown, dArmDown, 0);
                setArmControls(Constants.DEPOSIT_ARM_HOME);
            } else if (getArmVelocity() > 3 && armHome()) { // constant power to make sure arm does all the way home
                armMotor.setPower(-0.15);
            } else if (armSlidesHome()){
                armMotor.setPower(0);
            }
            if (target != Robot.DepositTarget.allianceHigh || !isAuto || getArmPosition() <= Constants.ARM_ON_HUB_THRESHOLD) {
                if (intakeTransfer && slidesHome()) slidesMotor.setPower(-0.25); // constant power so slides dont come out when robot slowing down
                else setSlidesControls();
            }
        } else {
            setSlidesInches(slidesDist); // Reset target every update to change with offset
            setArmTarget(targetArmPosNoOffset);
            // arm out first if low or mid goal
            if (!(target == Robot.DepositTarget.allianceLow || target == Robot.DepositTarget.allianceMid) || armAtPosPercent(0.75)) {
                setSlidesControls();
            }
            // midway arm pos if high or duck
            if ((target == Robot.DepositTarget.allianceHigh || target == Robot.DepositTarget.duck) && !slidesAtPosPercent(0.9)) {
                setArmControls(Constants.DEPOSIT_ARM_MIDWAY);
            } else {
                setArmControls();
            }
        }

        lastArmPos = getArmPosition();
        lastSlidesPos = getSlidesPosition();
    }

    // Arm
    public void setArmControls(int targetArmPos) {
        double targetTicks = Math.min(Math.max(targetArmPos, Constants.DEPOSIT_ARM_HOME), Constants.DEPOSIT_ARM_LOW);
        double currentTicks = getArmPosition();
        armErrorChange = targetTicks - currentTicks - armError;
        armError = targetTicks - currentTicks;

        double fArm = fGravity * Math.cos(getArmAngle());
        armMotor.setPower(pArm * armError + dArm * armErrorChange + fArm);

        Log.w("deposit-log", "arm set to: " + targetArmPos + ", current position: " + getArmPosition() + ", power " + (pArm * armError + dArm * armErrorChange + fArm));
    }

    public void setArmControls() {
        setArmControls(targetArmPos);
    }

    public void setArmTarget(int targetPos) {
        targetArmPos = Math.min(Math.max(targetPos + armOffset, 0), Constants.DEPOSIT_ARM_LOW);
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

    public boolean armHome() {
        return Math.abs(getArmPosition() - Constants.DEPOSIT_ARM_HOME) < DEPOSIT_ARM_ERROR_THRESHOLD;
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

    // Slides
    public void setSlidesControls(int targetSlidesPos) {
        slidesMotor.setTargetPosition(targetSlidesPos);
        slidesMotor.setPower(DEPOSIT_SLIDES_MAX_POWER);
        Log.w("deposit-log", "slides set to: " + targetSlidesPos + ", current position: " + getSlidesPosition());
    }

    public void setSlidesControls() {
        setSlidesControls(targetSlidesTicks);
    }

    public void setSlidesInches(double inches) {
        setSlidesTarget((int) Math.round(inches * DEPOSIT_SLIDES_TICKS_PER_INCH + slidesOffset));
    }

    public void setSlidesTarget(int targetPos) {
        targetSlidesTicks = Math.min(Math.max(targetPos, 0), DEPOSIT_SLIDES_MAX_TICKS);
    }

    public int getSlidesPosition() {
        return slidesMotor.getCurrentPosition() + initialSlidesPos;
    }

    public double getSlidesDistInches() {
        return getSlidesPosition() / DEPOSIT_SLIDES_TICKS_PER_INCH;
    }

    public boolean slidesAtPos() {
        return Math.abs(getSlidesPosition() - targetSlidesTicks) < DEPOSIT_SLIDES_ERROR_THRESHOLD;
    }

    public boolean slidesAtPosPercent(double percent) {
        return getSlidesPosition() >= targetSlidesTicks * percent;
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
        setServoPosition(target != Robot.DepositTarget.duck ? Constants.DEPOSIT_HOLD_POS : Constants.DEPOSIT_DUCK_HOLD_POS);
    }

    public boolean depositCleared() {
        return getSlidesPosition() > Math.round(DEPOSIT_SLIDES_TICKS_PER_INCH * Constants.SLIDES_DISTANCE_CLEAR);
    }

    public boolean armSlidesAtPose() {
        return armAtPos() && slidesAtPos();
    }

    public boolean armSlidesHome() {
        return getArmPosition() < 50 && slidesHome();
    }
}
