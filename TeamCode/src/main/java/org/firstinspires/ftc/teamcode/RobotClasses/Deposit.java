package org.firstinspires.ftc.teamcode.RobotClasses;

import static java.lang.Math.PI;

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

    private double lastServoPos = 69;

    public static double DEPOSIT_SLIDES_TICKS_PER_INCH = 9.142857;
    public static int DEPOSIT_SLIDES_MAX_TICKS = (int) (25 * DEPOSIT_SLIDES_TICKS_PER_INCH);
    public int DEPOSIT_SLIDES_ERROR_THRESHOLD = 15;
    public static double ARM_TICKS_PER_RADIAN = 1120 / (2*PI);
    public double DEPOSIT_ARM_MAX_POWER = 0.7;
    public int DEPOSIT_ARM_ERROR_THRESHOLD = 100;

    // Slides PD
    public static double pSlides = 50;

    public int targetSlidesTicks;

    private static final double maxSlidesDistBeforeLoweringArm = 2;

    public boolean home = true;
    public int targetArmPos;

    public DepositHeight targetHeight = DepositHeight.HOME;
    public enum DepositHeight {
        HOME, LOW, MID, HIGH
    }

    private double initialArmAngle = -0.646;

    // Arm PD
    double armErrorChange = 0, armError = 0;
    public static double pArmUp   = 0.004;
    public static double dArmUp   = 0.002;
    public static double pArmDown = 0.0015;
    public static double dArmDown = 0.0006;

    public static double pArm = pArmUp;
    public static double dArm = dArmUp;
    public static double fArm = 0;

    public static double fGravity = 0.05;

    public static boolean useMidwayArmPos = true;

    public Deposit(LinearOpMode op, boolean isAuto) {
        this(op, isAuto, PI/2);
    }

    public Deposit(LinearOpMode op, boolean isAuto, double initialRobotTheta) {
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

    public void setDepositControls(int targetArmPos, double slidesDist) {
        home = false;
        setArmPIDCoefficients(Deposit.pArmUp, Deposit.dArmUp);
        setArmTarget(targetArmPos);
        setSlidesTarget((int) Math.round(slidesDist * DEPOSIT_SLIDES_TICKS_PER_INCH));
    }

    public void update() {
        // Move Arm
        if (home) {
            if (getSlidesDistInches() < maxSlidesDistBeforeLoweringArm) {
                setArmControls();
            } else {
                setArmControls(Constants.DEPOSIT_ARM_MIDWAY);
            }
        } else {
            if (slidesAtPos() || !useMidwayArmPos) {
                setArmControls();
            } else {
                setArmControls(Constants.DEPOSIT_ARM_MIDWAY);
            }
        }

        // Move Slides
        // Cap Slides Extension Distance When Extending to the Side to Prevent Tipping
        // targetSlidesTicks = (int) Math.min(targetSlidesTicks, Constants.DEPOSIT_SLIDES_SIDE_EXTENSION_LIMIT_IN * Constants.DEPOSIT_SLIDES_TICKS_PER_INCH * Math.abs(1/Math.cos(getTurretTheta())));
        setSlidesControls();
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
        Robot.log("arm error:" + Math.abs(getArmPosition() - targetArmPos));
        return Math.abs(getArmPosition() - targetArmPos) < DEPOSIT_ARM_ERROR_THRESHOLD;
    }

    public boolean armHome() {
        return Math.abs(getArmPosition() - Constants.DEPOSIT_ARM_HOME) < DEPOSIT_ARM_ERROR_THRESHOLD;
    }

    public double getArmAngle() {
        return getArmPosition() / ARM_TICKS_PER_RADIAN + initialArmAngle;
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
        Robot.log("slides error:" + Math.abs(getSlidesPosition() - targetSlidesTicks));
        return Math.abs(getSlidesPosition() - targetSlidesTicks) < DEPOSIT_SLIDES_ERROR_THRESHOLD;
    }

    public boolean slidesAtPosPercent(double percent) {
        return getSlidesPosition() > targetSlidesTicks * percent;
    }

    public boolean slidesHome() {
        return Math.abs(getSlidesPosition()) < DEPOSIT_ARM_ERROR_THRESHOLD;
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
