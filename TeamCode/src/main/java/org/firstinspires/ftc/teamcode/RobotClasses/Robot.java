package org.firstinspires.ftc.teamcode.RobotClasses;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawDrivetrain;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawField;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawRobot;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;
import static java.lang.Math.sin;

import android.annotation.SuppressLint;
import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Debug.Logger;
import org.firstinspires.ftc.teamcode.Pathing.Pose;
import org.firstinspires.ftc.teamcode.Pathing.Target;
import org.firstinspires.ftc.teamcode.RobotClasses.whitetapedetectionstuff.TapeDetector;

import java.util.List;

@SuppressWarnings("FieldCanBeLocal")
public class Robot {

    // Robot Classes
    public Drivetrain drivetrain;
    public Intake intake;
    public Deposit deposit;
    public Carousel carousel;
    public TapeDetector tapeDetector;
    public Logger logger;

    private ElapsedTime profiler;
    private List<LynxModule> allHubs;
    private VoltageSensor battery;
    private boolean startVoltTooLow = false;

    // Class Constants
    private final int loggerUpdatePeriod = 2;
    private final int sensorUpdatePeriod = 3;
    private final double xyTolerance = 1;
    private final double thetaTolerance = PI/35;
    private final double slidesRetractMinDist = 6;
    public final static double[] cameraRelativeToRobot = new double[]{1,3};

    // State Variables
    private final boolean isAuto;
    public final boolean isRed;
    private boolean firstLoop = true;
    private int loopCounter = 0;

    public int cycles = 0;
    public double cycleTotal;
    public double lastCycleTime;
    public double longestCycle = 0;

    public boolean slidesMoving = false;
    public boolean intakeFull = false;
    public Deposit.DepositHeight depositHeight = Deposit.DepositHeight.HOME;
    public double intakePower = 0;
    private double[] scoringPos;

    public boolean depositToHome;
    public Deposit.DepositHeight depositTargetHeight;
    public boolean allianceHub;

    private boolean depositReadyToScore = false;
    private boolean depositReadyToReturn = false;
    public Deposit.DepositHeight depositMoveApproval = Deposit.DepositHeight.UNDEFINED;
    public boolean depositScoreApproval = false;

    // Time and Delay Variables
    public double curTime;

    // Motion Variables
    public double x, y, theta, vx, vy, w;
    private double prevX, prevY, prevTheta, prevVx, prevVy, prevW, prevTime, ax, ay, a;
    public double startTime;

    // OpMode Stuff
    private LinearOpMode op;

    //Config
    public boolean useTapeDetector;

    // Constructor
    public Robot(LinearOpMode op, double x, double y, double theta, boolean isAuto, boolean isRed) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.op = op;
        this.isAuto = isAuto;
        this.isRed = isRed;

        drivetrain = new Drivetrain(op, x, y, theta);
        intake = new Intake(op);
        deposit = new Deposit(op, isAuto);
        carousel = new Carousel(op);
        logger = new Logger();
        tapeDetector = new TapeDetector(op);

        useTapeDetector = isAuto;

        profiler = new ElapsedTime();

        allHubs = op.hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        battery = op.hardwareMap.voltageSensor.iterator().next();
        log("Battery Voltage: " + battery.getVoltage() + "v");
        if (battery.getVoltage() < 12.4) {
            startVoltTooLow = true;
        }

        drawField();
        drawRobot(this);
        sendPacket();
    }

    // Stop logger and t265
    public void stop() {
        logger.stopLogging();
    }

    // Reset Odometry
    public void resetOdo(double x, double y, double theta) {
        drivetrain.resetOdo(x, y, theta);
    }

    public void update() {
        loopCounter++;
        profiler.reset();
        curTime = System.currentTimeMillis();

        // Track time after start
        if (firstLoop) {
            startTime = curTime;
            lastCycleTime = curTime;
            firstLoop = false;
        }

        /*if (loopCounter % sensorUpdatePeriod == 0){
            slidesMoving = deposit.slidesMoving();
            intakeFull = intake.intakeFull();
            depositHeight = deposit.getTargetHeight();
            intakePower = intake.getLastIntakePow();
        }

        //State Controller
        if (depositHeight == depositHeight.HOME && !slidesMoving && !intakeFull){
            if(isAuto){
                intake.on();
            }
        }
        else if (depositHeight == depositHeight.HOME && intakeFull){
            intake.off();
            deposit.hold();
            if (depositMoveApproval != Deposit.deposit_height.UNDEFINED) {
                deposit.moveSlides(1, depositMoveApproval);
                depositMoveApproval = Deposit.deposit_height.UNDEFINED;
                depositReadyToScore = true;
            }
        }
        else if (depositReadyToScore && depositScoreApproval && depositHeight != Deposit.deposit_height.CAP && !slidesMoving && intakeFull){
            depositReadyToScore = false;
            depositScoreApproval = false;
            deposit.open();
            depositReadyToReturn = true;
            scoringPos[0] = x;
            scoringPos[1] = y;

        }
        else if (depositReadyToReturn && !slidesMoving && Math.sqrt(Math.pow(x-scoringPos[0],2)+Math.pow(y-scoringPos[1],2))>slidesRetractMinDist){
            depositReadyToReturn = false;
            deposit.close();
            deposit.moveSlides(1, Deposit.deposit_height.HOME);
        }*/

        // Update Position
        drivetrain.updatePose();

        // Calculate Motion Info
        double timeDiff = curTime / 1000 - prevTime;
        x = drivetrain.x;
        y = drivetrain.y;
        theta = drivetrain.theta;
        vx = (x - prevX) / timeDiff; vy = (y - prevY) / timeDiff; w = (theta - prevTheta) / timeDiff;
        ax = (vx - prevVx) / timeDiff; ay = (vy - prevVy) / timeDiff; a = (w - prevW) / timeDiff;

        // Remember Previous Motion Info
        prevX = x; prevY = y;
        prevTheta = theta;
        prevTime = curTime / 1000;
        prevVx = vx; prevVy = vy; prevW = w;

        //Update Tape Detector
        if (useTapeDetector){
            double[] resetOdoCoords = tapeDetector.update(drivetrain.x, drivetrain.y, drivetrain.theta);
            resetOdo(drivetrain.x, resetOdoCoords[0], resetOdoCoords[1]);
        }

        //Update depositor (arm + turret)
        if (depositToHome){
            deposit.setControlsHome();
        } else {
            double lockTheta;
            double slidesDist;

            if (allianceHub && isRed){
                lockTheta = atan2(y - 60, x - 96);
                slidesDist = hypot(y - 60, x - 96);
            } else if (allianceHub && !isRed){
                lockTheta = atan2(y - 60, x - 48);
                slidesDist = hypot(y - 60, x - 48);
            } else {
                lockTheta = atan2(y - 120, x - 72);
                slidesDist = hypot(y - 120, x - 72);
            }

            if (depositTargetHeight == Deposit.DepositHeight.LOW){
                deposit.setControlsDepositing(lockTheta, Constants.DEPOSIT_ARM_LOW_GOAL, slidesDist - Constants.ARM_DIST_PLUS_DIST_FROM_END_OF_SLIDES_TO_ROBOT_CENTER_LOW_GOAL);
            } else if (depositTargetHeight == Deposit.DepositHeight.MID){
                deposit.setControlsDepositing(lockTheta, Constants.DEPOSIT_ARM_MID_GOAL, slidesDist - Constants.ARM_DIST_PLUS_DIST_FROM_END_OF_SLIDES_TO_ROBOT_CENTER_MID_GOAL);
            } else if (depositTargetHeight == Deposit.DepositHeight.TOP){
                deposit.setControlsDepositing(lockTheta, Constants.DEPOSIT_ARM_TOP_GOAL, slidesDist - Constants.ARM_DIST_PLUS_DIST_FROM_END_OF_SLIDES_TO_ROBOT_CENTER_TOP_GOAL);
            } else {
                deposit.setControlsDepositing(lockTheta, Constants.DEPOSIT_ARM_CAP, slidesDist - Constants.ARM_DIST_PLUS_DIST_FROM_END_OF_SLIDES_TO_ROBOT_CENTER_CAP_GOAL);
            }
        }
        deposit.update(theta, drivetrain.commandedW);

        profile(1);

        // Log Data
        if (loopCounter % loggerUpdatePeriod == 0) {
            logger.logData(curTime - startTime, x, y, theta, vx, vy, w, ax, ay, a, deposit.targetHeight, cycles, cycleTotal / cycles);
        }

        profile(2);

        // Dashboard Telemetry
        if (startVoltTooLow) {
            addPacket("0", "Starting Battery Voltage < 12.4!!!!");
        }
        addPacket("1 X", round(x));
        addPacket("2 Y", round(y));
        addPacket("3 Theta", round(theta));
        addPacket("4 VX", round(vx));
        addPacket("5 VY", round(vy));
        addPacket("6 W", round(w));
        addPacket("7 Slides", deposit.targetHeight);
        addPacket("8 Run Time", (curTime - startTime) / 1000);
        addPacket("9 Update Frequency (Hz)", round(1 / timeDiff));
        if (!isAuto) {
            addPacket("Cycle Time", (curTime - lastCycleTime) / 1000);
            addPacket("Average Cycle Time", round(cycleTotal / cycles));
            addPacket("Cycles", cycles);
        }

        // Dashboard Drawings
        drawField();
        drawRobot(this);
        sendPacket();

        // Clear bulk cache
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        profile(3);
    }

    public void markCycle() {
        double cycleTime = (curTime - lastCycleTime) / 1000;
        cycleTotal += cycleTime;
        cycles++;
        Log.w("cycle-log", "Cycle " + cycles + ": " + cycleTime + "s");
        if (cycleTime > longestCycle) {
            longestCycle = cycleTime;
        }
        lastCycleTime = curTime;
    }

    // Set target point (velocity specification, custom b and zeta values)
    public void setTargetPoint(double xTarget, double yTarget, double thetaTarget, double vxTarget, double vyTarget, double wTarget, double xKp, double yKp, double thetaKp, double xKd, double yKd, double thetaKd) {
        // Make Sure thetaTarget is Between 0 and 2pi
        thetaTarget = thetaTarget % (2*PI);
        if (thetaTarget < 0) {
            thetaTarget += 2*PI;
        }

        // Picking the Smaller Distance to Rotate
        double thetaControl;
        if (abs(theta - thetaTarget) > PI) {
            thetaControl = abs(theta - thetaTarget) / (theta - thetaTarget) * (abs(theta - thetaTarget) - 2*PI);
        } else {
            thetaControl = theta - thetaTarget;
        }

        drawDrivetrain(xTarget, yTarget, thetaTarget, "blue");

        drivetrain.setGlobalControls(xKp * (xTarget - x) + xKd * (vxTarget - vx), yKp * (yTarget - y) + yKd * (vyTarget - vy), thetaKp * (-thetaControl) + thetaKd * (wTarget - w));
    }

    // Set target point (default Kp and Kv gains)
    public void setTargetPoint(double xTarget, double yTarget, double thetaTarget) {
        setTargetPoint(xTarget, yTarget, thetaTarget, 0, 0, 0, Drivetrain.xKp, Drivetrain.yKp, Drivetrain.thetaKp, Drivetrain.xKd, Drivetrain.yKd, Drivetrain.thetaKd);
    }

    // Set target point (using pose, velocity specification, default Kp and Kv gains)
    public void setTargetPoint(Pose pose) {
        setTargetPoint(pose.x, pose.y, pose.theta, pose.vx, pose.vy, pose.w, Drivetrain.xKp, Drivetrain.yKp, Drivetrain.thetaKp, Drivetrain.xKd, Drivetrain.yKd, Drivetrain.thetaKd);
    }

    // Set target point (using pose, custom theta and omega, default Kp and Kv gains)
    public void setTargetPoint(Pose pose, double theta, double w) {
        setTargetPoint(pose.x, pose.y, theta, pose.vx, pose.vy, w, Drivetrain.xKp, Drivetrain.yKp, Drivetrain.thetaKp, Drivetrain.xKd, Drivetrain.yKd, Drivetrain.thetaKd);
    }

    // Set target point (using target object)
    public void setTargetPoint(Target target) {
        Pose pose = target.getPose();
        setTargetPoint(pose.x, pose.y, pose.theta, pose.vx, pose.vy, pose.w, target.xKp(), target.yKp(), target.thetaKp(), target.xKd(), target.yKd(), target.thetaKd());
    }

    // Check if robot is at a certain point/angle (default tolerance)
    public boolean isAtPose(double targetX, double targetY) {
        return isAtPose(targetX, targetY, 0, xyTolerance, xyTolerance, 2*PI);
    }

    public boolean isAtPose(double targetX, double targetY, double targetTheta) {
        return isAtPose(targetX, targetY, targetTheta, xyTolerance, xyTolerance, thetaTolerance);
    }

    // Check if robot is at a certain point/angle (custom tolerance)
    public boolean isAtPose(double targetX, double targetY, double xTolerance, double yTolerance) {
        return abs(x - targetX) < xTolerance && abs(y - targetY) < yTolerance;
    }
    public boolean isAtPose(double targetX, double targetY, double targetTheta, double xTolerance, double yTolerance, double thetaTolerance) {
        return abs(x - targetX) < xTolerance && abs(y - targetY) < yTolerance && abs(theta - targetTheta) < thetaTolerance;
    }

    public boolean notMoving() {
        return notMoving(3.0, 0.2);
    }

    public boolean notMoving(double xyThreshold, double thetaThreshold) {
        return (hypot(vx, vy) < xyThreshold && abs(w) < thetaThreshold);
    }

    private static double sinc(double x) {
        if (abs(x) < 1e-6) {
            return 1.0 - 1.0 / 6.0 * x * x;
        } else {
            return sin(x) / x;
        }
    }

    private static double signOf(double x) {
        if (abs(x) < 1e-6) {
            return 1;
        } else {
            return abs(x) / x;
        }
    }

    // Logging
    public static void log(String message) {
        Log.w("robot-log", message);
    }

    private void profile(int num) {
//        Log.w("profiler", num + ": " + profiler.milliseconds());
    }

    @SuppressLint("DefaultLocale")
    public static double round(double num) {
        return Double.parseDouble(String.format("%.5f", num));
    }

    //Set Depositor Controls
    public void setDepositToHome(){
        depositToHome = true;
        this.depositTargetHeight = Deposit.DepositHeight.HOME;
    }
    public void depositTrackAllianceHub(Deposit.DepositHeight depositTargetHeight){
        depositToHome = false;
        allianceHub = true;
        this.depositTargetHeight = depositTargetHeight;
    }
    public void depositTrackSharedHub(){
        depositToHome = false;
        allianceHub = false;
        this.depositTargetHeight = Deposit.DepositHeight.LOW;
    }

}
