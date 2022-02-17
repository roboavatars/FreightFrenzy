package org.firstinspires.ftc.teamcode.RobotClasses;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawDrivetrain;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawField;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawRobot;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.hypot;

import android.annotation.SuppressLint;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Debug.Logger;
import org.firstinspires.ftc.teamcode.Localization.TapeDetector.TapeDetector;
import org.firstinspires.ftc.teamcode.Pathing.Pose;
import org.firstinspires.ftc.teamcode.Pathing.Target;

import java.util.List;

@SuppressWarnings("FieldCanBeLocal")
@Config
public class Robot {

    // Robot Classes
    public Drivetrain drivetrain;
    public Intake intake;
    public Turret turret;
    public Deposit deposit;
    public Carousel carousel;
    public TapeDetector tapeDetector;
    public Logger logger;

    private final ElapsedTime profiler;
    private final List<LynxModule> allHubs;
    private final VoltageSensor battery;
    private boolean startVoltTooLow = false;

    // Class Constants
    private final int loggerUpdatePeriod = 2;
    private final int sensorUpdatePeriod = 15;
    private final double xyTolerance = 1;
    private final double thetaTolerance = PI / 35;
    public final static double[] cameraRelativeToRobot = new double[]{1, 3};

    // State Variables
    private final boolean isAuto;
    public final boolean isRed;
    private boolean firstLoop = true;
    private int loopCounter = 0;
    public String automationStep = "n/a";

    public boolean intakeFull;
    public boolean intakeStalling;
    public double turretGlobalTheta;

    // Deposit Tracking
    public boolean trackGoal = false;
    private boolean setDepositControlsHome = true;
    public double distToGoal;
    public double turretLockTheta;
    public double turretFF = 0;
    public double[] turretCenter = new double[2];
    public double[] redGoalCoords = new double[] {96, 60};
    public double[] blueGoalCoords = new double[] {48, 60};
    public double[] neutGoalCoords = new double[] {72, 120};

    // Switching Between Preset Turret and Deposit Scoring Positions
    public enum DepositTarget {
        allianceLow,
        allianceMid,
        allianceHigh,
        neutral,
        duck
    }
    public DepositTarget cycleHub = DepositTarget.allianceHigh;

    public boolean useTapeDetector = false;

    // Automation Variables
    public boolean depositApproval = false;
    public boolean intakeApproval = false;
    public boolean intakeTransfer = false;
    public boolean depositingFreight = false;
    public boolean intakeRev = false;
    private double intakeFlipTime = -1;
    private double depositOpenTime = -1;
    public boolean noExtend = false;

    // Cycle Tracker
    public int cycles = 0;
    public double cycleTotal;
    public double lastCycleTime;
    public double longestCycle = 0;

    // Time and Delay Variables
    public double curTime;
    public static int flipUpThreshold = 1000;
    public static int transferThreshold = 2000;
    public static int releaseThreshold = 500;

    public double stallStartTime = -1;
    public static int stallThreshold = 1000;
    public double automationStepTime;

    // Motion Variables
    public double x, y, theta, vx, vy, w;
    private double prevX, prevY, prevTheta, prevVx, prevVy, prevW, prevTime, ax, ay, a;
    public double startTime;

    // OpMode Stuff
    private final LinearOpMode op;

    // Constructor
    public Robot(LinearOpMode op, double x, double y, double theta, boolean isAuto, boolean isRed) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.op = op;
        this.isAuto = isAuto;
        this.isRed = isRed;

        // init subsystems
        drivetrain = new Drivetrain(op, x, y, theta);
        intake = new Intake(op, isAuto);
        turret = new Turret(op, isAuto, theta);
        deposit = new Deposit(op, isAuto);
        carousel = new Carousel(op);
        logger = new Logger();
        tapeDetector = new TapeDetector(op);

        // set up bulk read
        allHubs = op.hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // low voltage warning
        battery = op.hardwareMap.voltageSensor.iterator().next();
        log("Battery Voltage: " + battery.getVoltage() + "v");
        if (battery.getVoltage() < 12.4) {
            startVoltTooLow = true;
        }
        profiler = new ElapsedTime();

        // initial dashboard drawings
        drawField();
        drawRobot(this);
        sendPacket();
    }

    // Stop logger
    public void stop() {
        logger.stopLogging();
    }

    // Reset Odometry
    public void resetOdo(double x, double y, double theta) {
        drivetrain.resetOdo(x, y, theta);
    }

    public void update() {
        // Don't check states every loop
        if (loopCounter % sensorUpdatePeriod == 0) {
            intakeFull = intake.intakeFull();
            intakeStalling = intake.checkIfStalling();
        }

        loopCounter++;
        profiler.reset();
        curTime = System.currentTimeMillis();

        // Track time after start
        if (firstLoop) {
            startTime = curTime;
            lastCycleTime = curTime;
            firstLoop = false;
        }

        // Auto-intaking
        if (!intakeTransfer && !depositingFreight && intake.slidesIsHome() && (/*y > 105 || */intakeApproval)) {
            if (!noExtend) intake.extend();
            else intake.extend(Constants.INTAKE_HOME_POS);
            intake.on();
            intakeTransfer = true;
            intakeApproval = false;
            automationStepTime = curTime;
            automationStep("Intake Extend/On");
        } else if (intakeTransfer) {
            if (intakeFull && !intake.slidesIsHome() && intakeFlipTime == -1) {
                intake.off();
                intake.home();
                automationStep("Intake Home/Off");
            } else if (intakeFull && intake.slidesIsHome() && intakeFlipTime == -1) {
                intake.flipUp();
                intake.setPower(0.25);
                intakeFlipTime = curTime;
                automationStep("Intake Flip Up");
            } else if (intakeFull && intake.slidesIsHome() && curTime - intakeFlipTime > flipUpThreshold && !intakeRev) {
                intake.reverse();
                intakeRev = true;
                automationStep("Transfer Block");
            } else if (!intakeFull && intake.slidesIsHome() && curTime - intakeFlipTime > transferThreshold && intakeRev) {
                deposit.hold();
                intake.off();
                intake.flipDown();
                automationStep("Intake Transfer Done");

                intakeTransfer = false;
                intakeFlipTime = -1;
                intakeRev = false;
                depositingFreight = true;
            }
        }

        // Auto-depositing
        if (depositingFreight) {
            if ((!isAuto && y <= 100) || (isAuto && y <= 100 && Math.abs(theta - PI / 2) < PI / 10) /*&& notMoving() && turret.turretAtPos()*/) {
                if (deposit.armSlidesHome() && depositOpenTime == -1) {
                    depositScore();
                    turretScore();
                    automationStep("Extend Slides/Arm");
                } else if (!deposit.armSlidesHome() && deposit.armSlidesAtPose() && depositOpenTime == -1 && (depositApproval && (!isAuto || deposit.getArmVelocity() < 5))) {
                    deposit.open();
                    depositOpenTime = curTime;
                    automationStep("Score Freight");
                } else if (!deposit.armSlidesHome() && deposit.armSlidesAtPose() && depositOpenTime != -1 && curTime - depositOpenTime > releaseThreshold) {
                    depositHome();
                    turretHome();

                    markCycle();
                    automationStep("Home Slides/Arm, Deposit Cycle Done");

                    depositApproval = false;
                    depositOpenTime = -1;
                    depositingFreight = false;
                }
            } else {
                if (y > 87) log("Waiting for dt pose");
                if (!notMoving()) log("Robot is moving");
                if (!turret.turretAtPos()) log("Waiting for turret align");
            }
        }

        if (intakeTransfer && !intakeFull && !intake.slidesIsHome()) {
            if (!intakeStalling) stallStartTime = -1;
            if (intakeStalling && stallStartTime == -1) { // 1
                stallStartTime = curTime;
            } else if (!intakeStalling && curTime - stallStartTime > stallThreshold + 500) { // 2
                intake.off();
            } else if (intakeStalling && curTime - stallStartTime > stallThreshold) { // 3
                intake.reverse();
            }
        }

        // Update Turret
        if (trackGoal) updateTrackingMath();
        turret.update(deposit.slidesHome(), theta, deposit.getSlidesDistInches(), turretFF);

        // Update Deposit
        if (trackGoal && !setDepositControlsHome) depositScore();
        deposit.update();

        // Update Position
        drivetrain.updatePose();

        // Calculate Motion Info
        double timeDiff = curTime / 1000 - prevTime;
        x = drivetrain.x;
        y = drivetrain.y;
        theta = drivetrain.theta;
        vx = (x - prevX) / timeDiff;
        vy = (y - prevY) / timeDiff;
        w = (theta - prevTheta) / timeDiff;
        ax = (vx - prevVx) / timeDiff;
        ay = (vy - prevVy) / timeDiff;
        a = (w - prevW) / timeDiff;

        // Remember Previous Motion Info
        prevX = x;
        prevY = y;
        prevTheta = theta;
        prevTime = curTime / 1000;
        prevVx = vx;
        prevVy = vy;
        prevW = w;

        /*
        // Update Tape Detector
        if (useTapeDetector) {
            double[] resetOdoCoords = tapeDetector.update(drivetrain.x, drivetrain.y, drivetrain.theta);
            resetOdo(drivetrain.x, resetOdoCoords[0], resetOdoCoords[1]);
        }
         */

        // Log Data
        if (loopCounter % loggerUpdatePeriod == 0) {
            logger.logData(curTime - startTime, x, y, theta, vx, vy, w, ax, ay, a,
                    turretGlobalTheta, deposit.getSlidesDistInches(), cycleHub, intake.slidesIsHome(), cycles, cycleTotal / cycles);
        }

        // Dashboard Telemetry
        if (startVoltTooLow) {
            addPacket("0", "Starting Battery Voltage < 12.4!!!!");
        }
        addPacket("1 X", round(x));
        addPacket("2 Y", round(y));
        addPacket("3 Theta", round(theta));
        addPacket("4 Turret Theta", round(turretGlobalTheta));
        addPacket("5 Deposit Level", cycleHub.name().toLowerCase());
        addPacket("7 Automation Step", automationStep);
        addPacket("8 Run Time", (curTime - startTime) / 1000);
        addPacket("9 Update Frequency (Hz)", round(1 / timeDiff));
        addPacket("Intake Full", intakeFull);
        addPacket("Intake Stalling", intakeStalling);
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
    }

    // Cancel auto-intaking/depositing
    public void cancelAutomation() {
        intake.off();
        intake.home();
        intake.flipDown();
        deposit.open();
        depositHome();
        turretHome();

        intakeApproval = false;
        intakeTransfer = false;
        intakeFlipTime = -1;
        intakeRev = false;
        depositApproval = false;
        depositOpenTime = -1;
        depositingFreight = false;
        automationStep("Automation Cancelled");
    }

    // Set Arm + Slides Control
    public void depositScore() {
        setDepositControlsHome = false;

        double slidesDist = 0;
        if (trackGoal) {
            updateTrackingMath();
            slidesDist = distToGoal - Constants.ARM_DISTANCE;
        } else {
            if (cycleHub == DepositTarget.allianceLow) {
                slidesDist = Constants.SLIDES_DISTANCE_LOW;
            } else if (cycleHub == DepositTarget.allianceMid) {
                slidesDist = Constants.SLIDES_DISTANCE_MID;
            } else if (cycleHub == DepositTarget.allianceHigh) {
                slidesDist = Constants.SLIDES_DISTANCE_HIGH;
            } else if (cycleHub == DepositTarget.duck) {
                slidesDist = Constants.SLIDES_DISTANCE_DUCK;
            }
        }
        if (cycleHub == DepositTarget.neutral) {
            slidesDist = 0;
        }
        deposit.setDepositControls(cycleHub, slidesDist);
    }

    // Arm + Slides Home
    public void depositHome() {
        setDepositControlsHome = true;
        deposit.setDepositHome();
    }

    // Set Turret Controls
    public void turretScore() {
        if (trackGoal) {
            updateTrackingMath();
            turret.setTracking(turretLockTheta);
        } else {
            double depositTheta = 0;
            if (cycleHub == DepositTarget.allianceLow || cycleHub == DepositTarget.allianceMid || cycleHub == DepositTarget.allianceHigh) {
                depositTheta = Constants.TURRET_ALLIANCE_RED_CYCLE_THETA;
            } else if (cycleHub == DepositTarget.neutral) {
                depositTheta = Constants.TURRET_NEUTRAL_RED_CYCLE_THETA;
            } else if (cycleHub == DepositTarget.duck) {
                depositTheta = Constants.TURRET_DUCK_RED_CYCLE_THETA;
            }
            turret.setDepositing(isRed ? depositTheta * PI : (1 - depositTheta) * PI);
        }
    }

    // Turret Home
    public void turretHome() {
        turret.setHome();
    }

    // Turret auto lock and slide extend distance
    public void updateTrackingMath() {
        turretCenter[0] = x + Math.cos(theta) * Turret.TURRET_Y_OFFSET;
        turretCenter[1] = y + Math.sin(theta) * Turret.TURRET_Y_OFFSET;

        double goalX;
        double goalY;
        if (cycleHub == DepositTarget.neutral) {
            goalX = neutGoalCoords[0];
            goalY = neutGoalCoords[1];
        } else { // Alliance Hub
            if (isRed) {
                goalX = redGoalCoords[0];
                goalY = redGoalCoords[1];
            } else {
                goalX = blueGoalCoords[0];
                goalY = blueGoalCoords[1];
            }
        }
        goalX -= turretCenter[0];
        goalY -= turretCenter[1];

        distToGoal = hypot(goalX, goalY);

        // calculates ff for turret control (w + atan dot)
        turretFF = w + (goalX * vy - vx * goalY) / (goalX * goalX + goalY * goalY);

        turretLockTheta = PI + Math.atan2(goalY, goalX);
        turretLockTheta %= 2*PI;
        if (turretLockTheta < 0) turretLockTheta += 2*PI;
    }

    // Keep track of cycles
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
        thetaTarget = thetaTarget % (2 * PI);
        if (thetaTarget < 0) {
            thetaTarget += 2 * PI;
        }

        // Picking the Smaller Distance to Rotate
        double thetaControl;
        if (abs(theta - thetaTarget) > PI) {
            thetaControl = abs(theta - thetaTarget) / (theta - thetaTarget) * (abs(theta - thetaTarget) - 2 * PI);
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
        return isAtPose(targetX, targetY, 0, xyTolerance, xyTolerance, 2 * PI);
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

    // Logging
    public static void log(String message) {
        Log.w("robot-log", message);
    }

    private void profile(int num) {
//        Log.w("profiler", num + ": " + profiler.milliseconds());
    }

    public void automationStep(String step) {
        automationStep = step;
        log((curTime - automationStepTime) + "ms");
        log(automationStep);
        automationStepTime = curTime;
    }

    @SuppressLint("DefaultLocale")
    public static double round(double num) {
        return Double.parseDouble(String.format("%.5f", num));
    }
}

