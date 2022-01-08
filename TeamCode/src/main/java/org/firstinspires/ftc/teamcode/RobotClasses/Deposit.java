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
    private DcMotorEx depositor; ////////
    private Servo depositServo;
    private Servo teamMarkerServo;

    private DcMotorEx turretMotor;
    private Servo armServo1;
    private Servo armServo2;
    private DcMotorEx slidesMotor;

    private static double lastTargetPower = 0;
    private static int lastTargetPos = 0;
    private static double lastServoPos = 0;
    private static int offset = 0;

    public static double pTurret = 2.25;
    public static double dTurret = 5.5;
    public static double fTurret = -0.3;
    public double initialTheta;

    private double targetTheta;
    private double turretTheta;
    private double turretError;
    private double turretErrorChange;
    private double lockTheta;

    private static final double TICKS_PER_RADIAN = 414.4 / PI;
    private static final double TURRET_MIN_THETA = PI/2;
    private static final double TURRET_MAX_THETA = 3*PI/2;

    private static final double SLIDES_MIN_TICKS = 0;
    private static final double SLIDES_MAX_TICKS = 1000;
    private static final double maxSlidesDistBeforeLoweringArm = 2;

    private static boolean home = true;
    public static double targetArmPos;

    public int targetSlidesTicks;

    public enum DepositHeight {
        HOME, LOW, MID, TOP, CAP, UNDEFINED
    }

    public DepositHeight targetHeight = DepositHeight.HOME;

    public Deposit(LinearOpMode op, boolean isAuto) {
        depositor = op.hardwareMap.get(DcMotorEx.class, "depositor");

        //Deposit Servo
        depositServo = op.hardwareMap.get(Servo.class, "depositServo");
        if (isAuto) {
            hold();
        } else{
            close();
        }

        //Team Marker Servo
        teamMarkerServo = op.hardwareMap.get(Servo.class, "teamMarkerArm");
        teamMarkerServo.setPosition(Constants.TEAM_MARKER_HOME_POS);

        //Turret Motor
        turretMotor = op.hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Arm Servos
        armServo1 = op.hardwareMap.get(Servo.class, "arm1");
        armServo2 = op.hardwareMap.get(Servo.class, "arm2");

        //Slides Motor
        slidesMotor = op.hardwareMap.get(DcMotorEx.class, "depositSlides");
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setTargetPosition(0);
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set Initial Turret Theta
        initialTheta = Constants.TURRET_HOME_THETA;

        setControlsHome();

        op.telemetry.addData("Status", "Deposit Initialized");
    }

    // Turret + Arm
    public void setControlsHome (){
        home = true;
        targetTheta = Constants.TURRET_HOME_THETA;
        targetArmPos = Constants.DEPOSIT_ARM_HOME;
        targetSlidesTicks = 0;
    }

    public void setControlsDepositing (double lockTheta, double targetArmPos, double slidesDist) {
        home = false;
        this.lockTheta = lockTheta;
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
            setTurretThetaPD(targetTheta);
        } else {
            targetTheta = (lockTheta - robotTheta) % (2 * PI);
            if (targetTheta < 0) {
                targetTheta += 2 * PI;
            }
            setTurretThetaPDF(targetTheta, commandedW);
        }

        //Move Slides
        moveSlides(Constants.DEPOSIT_SLIDES_POWER);
    }

    //Turret
    public void setTurretThetaPDF(double theta, double commandedW) {
        double clippedTargetTheta = Math.min(Math.max(theta, TURRET_MIN_THETA), TURRET_MAX_THETA);
        turretTheta = getTurretTheta();
        turretErrorChange = clippedTargetTheta - turretTheta - turretError;
        turretError = clippedTargetTheta - turretTheta;

        setTurretPower(fTurret * commandedW + pTurret * turretError + dTurret * turretErrorChange);
    }
    public void setTurretThetaPD(double theta) {
        double clippedTargetTheta = Math.min(Math.max(theta, TURRET_MIN_THETA), TURRET_MAX_THETA);
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
        return targetTheta;
    }

    public double getTurretTheta() {
        return turretMotor.getCurrentPosition() / TICKS_PER_RADIAN + initialTheta;
    }

    //Arm
    public void moveArm(double targetArmPos) {
        armServo1.setPosition(Math.min(Math.max(targetArmPos, 0),1));
        armServo2.setPosition(Math.min(Math.max(1 - targetArmPos + Constants.DEPOSIT_ARM_SERVO_OFFSET, 0),1));
    }

    //Slides
    public void moveSlides(double power){
        slidesMotor.setPower(power);
        slidesMotor.setTargetPosition((int) Math.min(Math.max(targetSlidesTicks, SLIDES_MIN_TICKS), SLIDES_MAX_TICKS));
    }
    public double getSlidesDistTicks(){
        return slidesMotor.getCurrentPosition();
    }
    public double getSlidesDistInches(){
        return slidesMotor.getCurrentPosition() / Constants.DEPOSIT_SLIDES_TICKS_PER_INCH;
    }

    // Deposit
    private void depositSetPosition(double pos) {
        if (pos != lastServoPos) {
            depositServo.setPosition(pos);
            lastServoPos = pos;
        }
    }

    public void autoOpen() {
        depositSetPosition(Constants.DEPOSIT_AUTO_OPEN_POS);
    }

    public void open() {
        depositSetPosition(Constants.DEPOSIT_OPEN_POS);
    }

    public void hold() {
        depositSetPosition(Constants.DEPOSIT_HOLD_POS);
    }

    public void close() {
        depositSetPosition(Constants.DEPOSIT_CLOSE_POS);
    }

    // Team marker
    public void markerSetPosition(double pos) {
        if (pos != lastServoPos) {
            teamMarkerServo.setPosition(pos);
            lastServoPos = pos;
        }
    }

    public void markerArmDown() {
        markerSetPosition(Constants.TEAM_MARKER_DOWN_POS);
    }

    public void markerArmUp() {
        markerSetPosition(Constants.TEAM_MARKER_UP_POS);
    }



    //////////////////////////////////////
    public void moveSlides(double power, DepositHeight depositHeight) {
        depositor.setPower(power);
        if (depositHeight == depositHeight.HOME) {
            depositor.setTargetPosition(Constants.HOME);
            targetHeight = depositHeight.HOME;
        } else if (depositHeight == depositHeight.LOW) {
            depositor.setTargetPosition(Constants.LOW_GOAL);
            targetHeight = depositHeight.LOW;
        } else if (depositHeight == depositHeight.MID) {
            depositor.setTargetPosition(Constants.MID_GOAL);
            targetHeight = depositHeight.MID;
        } else if (depositHeight == depositHeight.TOP) {
            depositor.setTargetPosition(Constants.TOP_GOAL);
            targetHeight = depositHeight.TOP;
        } else if (depositHeight == depositHeight.CAP) {
            depositor.setTargetPosition(Constants.CAP);
            targetHeight = depositHeight.CAP;
        } else {
            depositor.setTargetPosition(0);
        }
    }

    public double getSlidesHeight() {
        return depositor.getCurrentPosition() * 0.043;
    }
    //////////////////////////////////////////



}
