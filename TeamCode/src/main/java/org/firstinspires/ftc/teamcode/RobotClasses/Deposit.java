package org.firstinspires.ftc.teamcode.RobotClasses;

import static java.lang.Math.PI;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class Deposit {
    private DcMotorEx depositor;
    private Servo depositServo;
    private Servo teamMarkerServo;

    private DcMotorEx turretMotor;
    private DcMotorEx armMotor;

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
    private static final double MIN_THETA = PI/2;
    private static final double MAX_THETA = 3*PI/2;

    private static boolean home = true;
    private static boolean armOverThreshold = false;
    public static int targetArmHeight;
    public static int overrideTargetArmHeight;
    private final int turretInsideBotThreshold = 10;


    public enum DepositHeight {
        HOME, LOW, MID, TOP, CAP, UNDEFINED
    }

    public DepositHeight targetHeight = DepositHeight.HOME;

    public Deposit(LinearOpMode op, boolean isAuto) {
        depositor = op.hardwareMap.get(DcMotorEx.class, "depositor");
        depositServo = op.hardwareMap.get(Servo.class, "depositServo");
        teamMarkerServo = op.hardwareMap.get(Servo.class, "teamMarkerArm");

//        turretMotor = op.hardwareMap.get(DcMotorEx.class, "turret");
//        armMotor = op.hardwareMap.get(DcMotorEx.class, "v4b");
        initialTheta = Constants.TURRET_HOME_THETA;

        setControlsHome();

        if (isAuto) {
            hold();
        } else{
            close();
        }
        teamMarkerServo.setPosition(Constants.TEAM_MARKER_HOME_POS);

        depositor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        depositor.setTargetPosition(0);
        depositor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        depositor.setTargetPosition(0);

        op.telemetry.addData("Status", "Deposit Initialized");
    }

    // Turret + Arm
    public void setControlsHome (){
        home = true;
        targetTheta = Constants.TURRET_HOME_THETA;
        targetArmHeight = Constants.DEPOSIT_ARM_HOME;
    }

    public void setControlsDepositing (double lockTheta, int targetArmHeight) {
        home = false;
        this.lockTheta = lockTheta;
        this.targetArmHeight = targetArmHeight;
    }

    public void update(double robotTheta, double commandedW){
        overrideTargetArmHeight = targetArmHeight;
        armOverThreshold = armMotor.getCurrentPosition() > Constants.DEPOSIT_ARM_THRESHOLD;
        if (!armOverThreshold){
            if (home && !turretThetaInsideBot(getTurretTheta())){
                overrideTargetArmHeight = Constants.DEPOSIT_ARM_THRESHOLD;
            } else if (!home && turretThetaInsideBot(getTurretTheta())){
                overrideTargetArmHeight = Constants.DEPOSIT_ARM_THRESHOLD;
            }
        }
        moveArm(Constants.Deposit_ARM_POWER);

        if (home) {
            setTurretThetaPD(targetTheta);
        } else {
            targetTheta = (lockTheta - robotTheta) % (2 * PI);
            if (targetTheta < 0) {
                targetTheta += 2 * PI;
            }
            if (turretThetaInsideBot(targetTheta)){
                if (targetTheta<Constants.TURRET_HOME_THETA){
                    targetTheta = Constants.TURRET_HOME_THETA - turretInsideBotThreshold;
                } else {
                    targetTheta = Constants.TURRET_HOME_THETA + turretInsideBotThreshold;
                }
            }
            setTurretThetaPDF(targetTheta, commandedW);
        }
    }

    //Turret
    public void setTurretThetaPDF(double theta, double commandedW) {
        double clippedTargetTheta = Math.min(Math.max(theta, MIN_THETA), MAX_THETA);
        turretTheta = getTurretTheta();
        turretErrorChange = clippedTargetTheta - turretTheta - turretError;
        turretError = clippedTargetTheta - turretTheta;

        setTurretPower(fTurret * commandedW + pTurret * turretError + dTurret * turretErrorChange);
    }
    public void setTurretThetaPD(double theta) {
        double clippedTargetTheta = Math.min(Math.max(theta, MIN_THETA), MAX_THETA);
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

    public boolean turretThetaInsideBot(double theta){
        return Math.abs(theta-Constants.TURRET_HOME_THETA) < turretInsideBotThreshold;
    }

    //Arm
    public void moveArm(double power) {
        armMotor.setPower(power);
        armMotor.setTargetPosition(overrideTargetArmHeight);
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
}
