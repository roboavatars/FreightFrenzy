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
    private DcMotorEx turretMotor;
    private DcMotorEx slidesMotor;
    private DcMotorEx armMotor;
    private Servo depositServo;

    private static double lastServoPos = 0;

    public double MAX_TURRET_POWER = 1;
    public double TURRET_MIN_THETA = 0;
    public double TURRET_MAX_THETA = PI;
    public double TURRET_TICKS_PER_RADIAN = 103.6 * 20 / (2*PI);
    public static double TURRET_Y_OFFSET = 2.06066;
    public double TURRET_ERROR_THRESHOLD = PI/40;
    public static double DEPOSIT_SLIDES_TICKS_PER_INCH = 9.142857;
    public static int DEPOSIT_SLIDES_MAX_TICKS = (int) (25 * DEPOSIT_SLIDES_TICKS_PER_INCH);
    public int DEPOSIT_SLIDES_ERROR_THRESHOLD = 10;
    public static double ARM_TICKS_PER_RADIAN = 1120 / (2*PI);
    public double DEPOSIT_ARM_MAX_POWER = 0.7;
    public int DEPOSIT_ARM_ERROR_THRESHOLD = 5;

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

    // Slides PD
    public static double pSlides = 50;

    public int targetSlidesTicks;
    private double slidesError = 0;

    private static final double maxSlidesDistBeforeLoweringArm = 2;

    public boolean home = true;
    public int targetArmPos;

    public DepositHeight targetHeight = DepositHeight.HOME;
    public enum DepositHeight {
        HOME, LOW, MID, HIGH
    }

    double initialArmAngle = -0.646;

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

    public Deposit(LinearOpMode op, boolean isAuto) {
        this(op, isAuto, PI/2);
    }

    public Deposit(LinearOpMode op, boolean isAuto, double initialRobotTheta) {
        // Deposit Servo
        depositServo = op.hardwareMap.get(Servo.class, "depositServo");
        if (isAuto) {
            close();
        } else {
            open();
        }

        // Turret Motor
        turretMotor = op.hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        // Set Initial Turret Theta
        initialTheta = initialRobotTheta;

        setDepositHome();

        op.telemetry.addData("Status", "Deposit Initialized");
    }

    // Slides + Arm
    public void setDepositHome() {
        home = true;
        setArmTarget(0);
//        setArmPIDCoefficients(pArmGoingDown);
        setSlidesTarget(0);
//        setSlidesPIDCoefficients(pSlidesRetract);
    }

    public void setDepositControls(int targetArmPos, double slidesDist) {
        home = false;
        setArmTarget(targetArmPos);
//        setArmPIDCoefficients(pArmGoingUp);
        setSlidesTarget((int) Math.round(slidesDist * DEPOSIT_SLIDES_TICKS_PER_INCH));
//        setSlidesPIDCoefficients(pSlidesExtend);
    }

    public void setTurretLockTheta(double lockTheta) {
        turretLockTheta = lockTheta;
    }

    public void turretHome() {
        turretTargetTheta = initialTheta;
    }

    public void update(double robotTheta, double turretFF) {
        // Move Arm
        if (home && getSlidesDistInches() > maxSlidesDistBeforeLoweringArm) {
            setArmControls(Constants.DEPOSIT_ARM_OVER_SLIDES_MOTOR);
        } else {
            setArmControls();
        }

        // Move Turret
        updateTurret(robotTheta, turretFF);

        // Move Slides
        // Cap Slides Extension Distance When Extending to the Side to Prevent Tipping
//        targetSlidesTicks = (int) Math.min(targetSlidesTicks, Constants.DEPOSIT_SLIDES_SIDE_EXTENSION_LIMIT_IN * Constants.DEPOSIT_SLIDES_TICKS_PER_INCH * Math.abs(1/Math.cos(getTurretTheta())));

        setSlidesControls();
    }

    public void updateTurret(double robotTheta, double turretFF) {
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

        setTurretPower(pTurret * turretError + dTurret * turretErrorChange + fwTurret * ff + fmoiTurret * getSlidesDistInches() * getSlidesDistInches());
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

    // Arm
    public void setArmControls(int targetArmPos) {

        fArm = fGravity * Math.cos(getArmAngle());

        double targetTicks = (int) Math.min(Math.max(targetArmPos, Constants.DEPOSIT_ARM_HOME), Constants.DEPOSIT_ARM_LOW);
        double currentTicks = getArmPosition();
        armErrorChange = targetTicks - currentTicks - armError;
        armError = targetTicks - currentTicks;

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

    public boolean armAtPos() {
        return Math.abs(getArmPosition() - targetArmPos) < DEPOSIT_ARM_ERROR_THRESHOLD;
    }

    public boolean armHome() {
        return Math.abs(getArmPosition() - Constants.DEPOSIT_ARM_OVER_SLIDES_MOTOR) < DEPOSIT_ARM_ERROR_THRESHOLD;
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

    public void close() {
        setServoPosition(Constants.DEPOSIT_CLOSE_POS);
    }

    public void hold() {
        setServoPosition(Constants.DEPOSIT_HOLD_POS);
    }

    public boolean atPose() {
        return turretAtPos() && armAtPos() && slidesAtPos();
    }

    public boolean armSlidesAtPose() {
        return armAtPos() && slidesAtPos();
    }

    public boolean armSlidesHome() {
        return armHome() && slidesHome();
    }
}
