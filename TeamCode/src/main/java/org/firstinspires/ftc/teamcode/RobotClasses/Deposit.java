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
    public static double pSlides = 0.02;
    public static double dSlides = 0;

    private int targetSlidesTicks;
    private double slidesError = 0;
    private double slidesErrorChange;

    // Arm PD
    public static double pArm = 0.05;
    public static double dArm = 0.001;

    private static final double maxSlidesDistBeforeLoweringArm = 2;

    public boolean home = true;
    public int targetArmPos;

    public enum DepositHeight {
        HOME, LOW, MID, HIGH
    }

    public DepositHeight targetHeight = DepositHeight.HOME;

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
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Slides Motor
        slidesMotor = op.hardwareMap.get(DcMotorEx.class, "depositSlides");
        slidesMotor.setDirection(DcMotorEx.Direction.REVERSE);
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slidesMotor.setTargetPosition(0);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        slidesMotor.setPositionPIDFCoefficients(pSlides);

        // Set Initial Turret Theta
        initialTheta = initialRobotTheta;

        setDepositHome();

        op.telemetry.addData("Status", "Deposit Initialized");
    }

    // Slides + Arm
    public void setDepositHome() {
        home = true;
        targetArmPos = Constants.DEPOSIT_ARM_TRANSFER;
        targetSlidesTicks = 0;
    }

    public void setDepositControls(int targetArmPos, double slidesDist) {
        home = false;
        this.targetArmPos = targetArmPos;
        targetSlidesTicks = (int) Math.round(slidesDist * Constants.DEPOSIT_SLIDES_TICKS_PER_INCH);
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
            setArmControls(Constants.DEPOSIT_ARM_TRANSFER);
        } else {
            setArmControls();
        }

        // Move Turret
        updateTurret(robotTheta, turretFF);

        // Move Slides
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

        setTurretPower(pTurret * turretError + dTurret * turretErrorChange + fwTurret * ff/* + fmoiTurret * getSlidesPosition()*/);
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
        armMotor.setTargetPosition(targetArmPos);
        armMotor.setPower(Constants.DEPOSIT_ARM_MAX_POWER);
    }

    public void setArmControls() {
        setArmControls(targetArmPos);
    }

    public double getArmPosition() {
        return armMotor.getCurrentPosition();
    }

    public boolean armAtPos() {
        return Math.abs(getArmPosition() - targetArmPos) < Constants.DEPOSIT_ARM_ERROR_THRESHOLD;
    }

    public boolean armHome() {
        return Math.abs(getArmPosition() - Constants.DEPOSIT_ARM_TRANSFER) < Constants.DEPOSIT_ARM_ERROR_THRESHOLD;
    }

    // Slides
    public void setSlidesControls(int targetSlidesPos) {
        double targetTicks = (int) Math.min(Math.max(targetSlidesPos, Constants.DEPOSIT_SLIDES_MIN_TICKS), Constants.DEPOSIT_SLIDES_MAX_TICKS);
        double currentTicks = getSlidesPosition();
        slidesErrorChange = targetTicks - currentTicks - slidesError;
        slidesError = targetTicks - currentTicks;

        slidesMotor.setPower(-(pSlides * slidesError + dSlides * slidesErrorChange)); //Power is negative because of the inversed turret slides motor wiring

    }

    public void setSlidesControls() {
        setSlidesControls(targetSlidesTicks);
    }

    public double getSlidesPosition() {
        return slidesMotor.getCurrentPosition();
    }

    public double getSlidesDistInches() {
        return getSlidesPosition() / Constants.DEPOSIT_SLIDES_TICKS_PER_INCH;
    }

    public boolean slidesAtPos() {
        return Math.abs(getSlidesPosition() - targetSlidesTicks) < Constants.DEPOSIT_SLIDES_ERROR_THRESHOLD;
    }

    public boolean slidesHome() {
        return Math.abs(getSlidesPosition()) < 0;
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
