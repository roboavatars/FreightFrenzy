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
    private final int sensorUpdatePeriod = 10;
    private final double xyTolerance = 1;
    private final double thetaTolerance = PI / 35;
    public final static double[] cameraRelativeToRobot = new double[]{1, 3};


    // State Variables
    private final boolean isAuto;
    public final boolean isRed;
    private boolean firstLoop = true;
    private int loopCounter = 0;
    public String automationStep = "n/a";

    public Deposit.DepositHeight depositTargetHeight = Deposit.DepositHeight.HIGH;

    public boolean intakeFull;
    public double turretGlobalTheta;
    public double slidesDist;

    //Switching Between Preset Turret and Deposit Scoring Positions
    public enum hub {
        allianceLow,
        allianceMid,
        allianceHigh,
        neutral,
        duck
    }
    public hub cycleHub = hub.allianceHigh;

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
        loopCounter++;
        profiler.reset();
        curTime = System.currentTimeMillis();

        // Track time after start
        if (firstLoop) {
            startTime = curTime;
            lastCycleTime = curTime;
            firstLoop = false;
        }

        // Don't check intake sensor every loop
        if (loopCounter % sensorUpdatePeriod == 0) {
            intakeFull = intake.intakeFull();
        }

        if (!intakeTransfer && !depositingFreight && intake.slidesIsHome() && (y > 105 || intakeApproval)) {
            if (!noExtend) intake.extend();
            else intake.extend(Constants.INTAKE_HOME_POS);
            intake.on();
            intakeTransfer = true;
            intakeApproval = false;
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
            log("full: " + intakeFull + ", home: " + intake.slidesIsHome() + ", dFlip: " + (curTime - intakeFlipTime) + "");
        }

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
                    intakeTransfer = false;
                    depositingFreight = false;
                }

                log("home: " + deposit.armSlidesHome() + ", atpos: " + deposit.armSlidesAtPose());
            } else {
                if (y > 87) log("Waiting for dt pose");
                if (!notMoving()) log("Robot is moving");
                if (!turret.turretAtPos()) log("Waiting for turret align");
            }
        }

        // Update Turret
        turret.update(deposit.slidesHome());

        //Update Deposit
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
                    turretGlobalTheta, slidesDist, depositTargetHeight, intake.slidesIsHome(), cycles, cycleTotal / cycles);
        }

        // Dashboard Telemetry
        if (startVoltTooLow) {
            addPacket("0", "Starting Battery Voltage < 12.4!!!!");
        }
        addPacket("1 X", round(x));
        addPacket("2 Y", round(y));
        addPacket("3 Theta", round(theta));
        addPacket("4 Turret Theta", round(turretGlobalTheta));
        addPacket("5 Deposit Level", depositTargetHeight.name().toLowerCase());
        addPacket("7 Automation Step", automationStep);
        addPacket("8 Run Time", (curTime - startTime) / 1000);
        addPacket("9 Update Frequency (Hz)", round(1 / timeDiff));
        addPacket("intake full, intake approval, deposit approval", intakeFull + " " + intakeApproval + " " + depositApproval);
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

    public void depositScore() {
        //Set Arm + Slides Controls
        if (cycleHub == hub.allianceLow) {
            Deposit.useMidwayArmPos = false;
            deposit.setDepositControls(Constants.DEPOSIT_ARM_LOW, Constants.SLIDES_DISTANCE_LOW);
        } else if (cycleHub == hub.allianceMid) {
            Deposit.useMidwayArmPos = false;
            deposit.setDepositControls(Constants.DEPOSIT_ARM_MID, Constants.SLIDES_DISTANCE_MID);
        } else if (cycleHub == hub.allianceHigh) {
            Deposit.useMidwayArmPos = true;
            deposit.setDepositControls(Constants.DEPOSIT_ARM_HIGH, Constants.SLIDES_DISTANCE_HIGH);
        } else if (cycleHub == hub.neutral) {
            Deposit.useMidwayArmPos = false;
            deposit.setDepositControls(Constants.DEPOSIT_ARM_LOW, 0);
        } else if (cycleHub == hub.duck){
            Deposit.useMidwayArmPos = true;
            deposit.setDepositControls(Constants.DEPOSIT_ARM_HIGH, Constants.SLIDES_DISTANCE_DUCK);
        }
    }

    public void depositHome() {
        deposit.setDepositHome();
    }

    public void turretScore() {
        //Set Turret Controls
        if (cycleHub == hub.allianceLow) {
            if (isRed) {
                turret.setDepositing(Constants.TURRET_ALLIANCE_RED_CYCLE_THETA * PI);
            } else {
                turret.setDepositing((1-Constants.TURRET_ALLIANCE_RED_CYCLE_THETA) * PI);
            }
        } else if (cycleHub == hub.allianceMid) {
            if (isRed) {
                turret.setDepositing(Constants.TURRET_ALLIANCE_RED_CYCLE_THETA * PI);
            } else {
                turret.setDepositing((1-Constants.TURRET_ALLIANCE_RED_CYCLE_THETA) * PI);
            }
        } else if (cycleHub == hub.allianceHigh) {
            if (isRed) {
                turret.setDepositing(Constants.TURRET_ALLIANCE_RED_CYCLE_THETA * PI);
            } else {
                turret.setDepositing((1-Constants.TURRET_ALLIANCE_RED_CYCLE_THETA) * PI);
            }
        } else if (cycleHub == hub.neutral) {
            if (isRed) {
                turret.setDepositing(Constants.TURRET_NEUTRAL_RED_CYCLE_THETA * PI);
            } else {
                turret.setDepositing((1-Constants.TURRET_NEUTRAL_RED_CYCLE_THETA) * PI);
            }
        } else if (cycleHub == hub.duck){
            if (isRed) {
                turret.setDepositing(Constants.TURRET_DUCK_RED_CYCLE_THETA * PI);
            } else {
                turret.setDepositing((1-Constants.TURRET_DUCK_RED_CYCLE_THETA) * PI);
            }
        }
    }

    public void turretHome() {
        turret.setHome();
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
        log(automationStep);
    }

    @SuppressLint("DefaultLocale")
    public static double round(double num) {
        return Double.parseDouble(String.format("%.5f", num));
    }
}

