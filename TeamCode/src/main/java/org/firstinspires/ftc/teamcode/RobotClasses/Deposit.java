package org.firstinspires.ftc.teamcode.RobotClasses;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.opencv.core.Mat;

@SuppressWarnings("FieldCanBeLocal")
@Config
public class Deposit {
    private DcMotorEx depositor; ////////
    private Servo depositServo;

    private DcMotorEx turretMotor;
    private DcMotorEx slidesMotor;
    private DcMotorEx armMotor;

    private static double lastServoPos = 0;

    //Turret PD and PDFF
    public static double pTurret = 2.25;
    public static double dTurret = 5.5;
    public static double fwTurret = -0.3;
    public static double fmoiTurret = 0;
    public double initialTheta;

    private double turretTargetTheta;
    private double turretTheta;
    private double turretError = 0;
    private double turretErrorChange;
    private double turretLockTheta;

    //Slides PD
    public static double pSlides = 0.2;
    public static double dSlides = 0;

    private int targetSlidesTicks;
    private double slidesError = 0;
    private double slidesErrorChange;

    //Arm PD
    public static double pArm = 0.07;
    public static double dArm = 0;

    private double armError = 0;
    private double armErrorChange;

    private static final double maxSlidesDistBeforeLoweringArm = 2;

    private static boolean home = true;
    public static double targetArmPos;

    public enum DepositHeight {
        HOME, LOW, MID, TOP
    }

    public DepositHeight targetHeight = DepositHeight.HOME;

    public Deposit(LinearOpMode op, boolean isAuto) {
        //Deposit Servo
        depositServo = op.hardwareMap.get(Servo.class, "depositServo");
        if (isAuto) {
            close();
        } else{
            open();
        }

        //Turret Motor
        turretMotor = op.hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Arm Servos
        armMotor = op.hardwareMap.get(DcMotorEx.class, "arm1");

        //Slides Motor
        slidesMotor = op.hardwareMap.get(DcMotorEx.class, "depositSlides");
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set Initial Turret Theta
        initialTheta = Constants.TURRET_HOME_THETA;

        setControlsHome();

        op.telemetry.addData("Status", "Deposit Initialized");
    }

    // Turret + Arm
    public void setControlsHome (){
        home = true;
        turretTargetTheta = Constants.TURRET_HOME_THETA;
        targetArmPos = Constants.DEPOSIT_ARM_HOME_TICKS;
        targetSlidesTicks = 0;
    }

    public void setControlsDepositing (double lockTheta, double targetArmPos, double slidesDist) {
        home = false;
        this.turretLockTheta = lockTheta;
        this.targetArmPos = targetArmPos;
        targetSlidesTicks = (int) Math.round(slidesDist * Constants.DEPOSIT_SLIDES_TICKS_PER_INCH);
    }

    public void update(double robotTheta, double commandedW){

        //Move Arm
        if (home && getSlidesDistInches() > maxSlidesDistBeforeLoweringArm){
            moveArm(Constants.DEPOSIT_ARM_OVER_MOTOR);
        } else {
            moveArm(targetArmPos);
        }

        //Move Turret
        if (home) {
            setTurretThetaPD(turretTargetTheta);
        } else {
            turretTargetTheta = (turretLockTheta - robotTheta) % (2 * PI);
            if (turretTargetTheta < 0) {
                turretTargetTheta += 2 * PI;
            }
            setTurretThetaPDFF(turretTargetTheta, commandedW);
        }

        //Move Slides
        slidesPD(Constants.DEPOSIT_SLIDES_POWER);
    }

    //Turret
    public void setTurretThetaPDFF(double theta, double commandedW) {
        double clippedTargetTheta = Math.min(Math.max(theta, Constants.TURRET_MIN_THETA), Constants.TURRET_MAX_THETA);
        turretTheta = getTurretTheta();
        turretErrorChange = clippedTargetTheta - turretTheta - turretError;
        turretError = clippedTargetTheta - turretTheta;

        setTurretPower(pTurret * turretError + dTurret * turretErrorChange + fwTurret * commandedW + fmoiTurret * getSlidesDistTicks());
    }
    public void setTurretThetaPD(double theta) {
        double clippedTargetTheta = Math.min(Math.max(theta, Constants.TURRET_MIN_THETA), Constants.TURRET_MAX_THETA);
        turretTheta = getTurretTheta();
        turretErrorChange = clippedTargetTheta - turretTheta - turretError;
        turretError = clippedTargetTheta - turretTheta;

        setTurretPower(pTurret * turretError + dTurret * turretErrorChange);
    }

    public void setTurretPower(double power) {
        turretMotor.setPower(Math.max(Math.min(power, Constants.MAX_TURRET_POWER), -Constants.MAX_TURRET_POWER));
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
        return turretMotor.getCurrentPosition() / Constants.TURRET_TICKS_PER_RADIAN + initialTheta;
    }

    public double getTurretError(){
        return turretError;
    }

    public boolean turretAtPos(){
        return Math.abs(turretError) < Constants.TURRET_ERROR_THRESHOLD;
    }

    //Arm
    public void moveArm(double targetArmPos) {
        double targetTicks = (int) Math.min(Math.max(targetArmPos, Constants.DEPOSIT_ARM_HOME_TICKS), Constants.DEPOSIT_ARM_LOW_GOAL_TICKS);
        double currentTicks = getArmTicks();
        armErrorChange = targetTicks - currentTicks - armError;
        armError = targetTicks - currentTicks;

        armMotor.setPower(-(pArm * armError + dArm * armErrorChange));
    }

    public double getArmTicks(){
        return armMotor.getCurrentPosition();
    }

    public boolean armAtPos(){
        return Math.abs(armError) < Constants.DEPOSIT_ARM_ERROR_THRESHOLD;
    }

    //Slides
    public void slidesPD(double power){
        double targetTicks = (int) Math.min(Math.max(targetSlidesTicks, Constants.DEPOSIT_SLIDES_MIN_TICKS), Constants.DEPOSIT_SLIDES_MAX_TICKS);
        double currentTicks = getSlidesDistTicks();
        slidesErrorChange = targetTicks - currentTicks - slidesError;
        slidesError = targetTicks - currentTicks;

        slidesMotor.setPower(-(pSlides * slidesError + dSlides * slidesErrorChange)); //Power is negative because of the inversed turret slides motor wiring
    }
    public double getSlidesDistTicks(){
        return slidesMotor.getCurrentPosition();
    }

    public double getSlidesDistInches(){
        return slidesMotor.getCurrentPosition() / Constants.DEPOSIT_SLIDES_TICKS_PER_INCH;
    }

    public boolean slidesAtPos(){
        return Math.abs(slidesError) < Constants.DEPOSIT_SLIDES_ERROR_THRESHOLD;
    }

    // Deposit
    private void depositSetPosition(double pos) {
        if (pos != lastServoPos) {
            depositServo.setPosition(pos);
            lastServoPos = pos;
        }
    }

    public void open() {
        depositSetPosition(Constants.DEPOSIT_OPEN_POS);
    }

    public void close() {
        depositSetPosition(Constants.DEPOSIT_CLOSE_POS);
    }

    public boolean atPose(){
        return turretAtPos() && armAtPos() && slidesAtPos();
    }
}
