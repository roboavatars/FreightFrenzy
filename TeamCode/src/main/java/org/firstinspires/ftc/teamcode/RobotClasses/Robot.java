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
import org.firstinspires.ftc.teamcode.Localization.TapeDetector.TapeDetector;
import org.firstinspires.ftc.teamcode.Pathing.Pose;
import org.firstinspires.ftc.teamcode.Pathing.Target;

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
    private final double turretTolerance = PI/50;
    public final static double[] cameraRelativeToRobot = new double[] {1, 3};
    public static double[] redGoalCoords = new double[] {96, 60};
    public static double[] blueGoalCoords = new double[] {48, 60};
    public static double[] neutGoalCoords = new double[] {72, 120};

    // State Variables
    private final boolean isAuto;
    public final boolean isRed;
    private boolean firstLoop = true;
    private int loopCounter = 0;

    public boolean intakeTransfer = false;
    public boolean depositingFreight = false;
    public boolean depositDone = false;
    public boolean intakeSlidesHome;
    public boolean intakeFull;

    // Cycle Tracker
    public int cycles = 0;
    public double cycleTotal;
    public double lastCycleTime;
    public double longestCycle = 0;

    public double turretTheta;
    public double depositSlidesDist = 0;

    public boolean turretHome = false;
    private Deposit.DepositHeight depositTargetHeight;
    public boolean allianceHub;
    public boolean depositApproval = false;

    public double turretGlobalTheta;
    public double lockTheta;
    public double slidesDist;

    // Time and Delay Variables
    public double curTime;
    private double intakeFlipTime;
    private double depositOpenTime;

    // Motion Variables
    public double x, y, theta, vx, vy, w;
    private double prevX, prevY, prevTheta, prevVx, prevVy, prevW, prevTime, ax, ay, a;
    public double startTime;

    // OpMode Stuff
    private LinearOpMode op;

    //Config
    public boolean useTapeDetector = false;
    public int transferThreshold = 1000;
    public int releaseThreshold = 1000;

    public String automationStep = "n/a";

    // Constructor
    public Robot(LinearOpMode op, double x, double y, double theta, boolean isAuto, boolean isRed) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.op = op;
        this.isAuto = isAuto;
        this.isRed = isRed;

        drivetrain = new Drivetrain(op, x, y, theta);
        intake = new Intake(op, isAuto);
        deposit = new Deposit(op, isAuto, theta);
        carousel = new Carousel(op);
        logger = new Logger();
        tapeDetector = new TapeDetector(op);

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

        if (loopCounter % sensorUpdatePeriod == 0) {
            intakeSlidesHome = intake.slidesIsHome();
            intakeFull = intake.intakeFull();
            turretTheta = deposit.getTurretTheta();
            depositSlidesDist = deposit.getSlidesDistInches();
        }

        /* automation planning
        if passing white line going toward warehouse:
        extend intake slides
        intake on

        if intake slides out, distance sensor thresh met:
        intake off
        retract slides

        if intake slides home, distance sensor thresh met:
        flip intake up

        if intake up, slides home, distance sensor thresh met, x ms after flip up:
        deposit servo down
        flip intake down
        auto-align turret

        if turret and robit at pos, arm/slides home:
        extend deposit slides
        extend arm

        if at pose, arm and slides at pos + driver approval:
        deposit servo up

        x ms after deposit:
        arm home
        retract slides

        after slides home:
        turret home
         */

        if (!intakeTransfer && intake.slidesIsHome() && y > 105) {
            intake.extend();
            intake.on();
            intakeTransfer = true;
            automationStep("Intake Extend/On");
        } else if (intakeTransfer) {
            if (intakeFull && !intake.slidesIsHome() && intakeFlipTime == -1) {
                intake.off();
                intake.home();
                automationStep("Intake Home/Off");
            } else if (intakeFull && intake.slidesIsHome() && intakeFlipTime == -1) {
                intake.flipUp();
                intake.setPower(-0.3);
                intakeFlipTime = curTime;
                automationStep("Intake Flip Up");
            } else if (!intakeFull && intake.slidesIsHome() && curTime - intakeFlipTime > transferThreshold) {
                deposit.close();
                intake.off();
                intake.flipDown();
                turretHome = false;
                automationStep("Intake Transfer Done");

                intakeFlipTime = -1;
                depositingFreight = true;
            }
        }

        if (depositingFreight) {
            if (!depositDone && isAtPose(x, 87) && deposit.turretAtPos()) {
                if (deposit.armSlidesHome()) {
                    depositAllianceHub(Deposit.DepositHeight.HIGH);
                    automationStep("Extend Slides/Arm");
                } else if (deposit.armSlidesAtPose() && depositOpenTime == -1) {
                    deposit.open();
                    depositOpenTime = curTime;
                    automationStep("Score Freight");
                } else if (deposit.armSlidesAtPose() && curTime - depositOpenTime > releaseThreshold && depositApproval) {
                    depositHome();
                    depositDone = true;
                    automationStep("Home Slides/Arm");
                }
            } else if (depositDone && deposit.armSlidesHome()) {
                turretHome = true;
                markCycle();
                automationStep("Deposit Cycle Done");

                depositOpenTime = -1;
                intakeTransfer = false;
                depositingFreight = false;
            } else {
                if (!depositDone) {
                    if (!isAtPose(x, 87)) log("Waiting for dt pose");
                    else if (!deposit.turretAtPos()) log("Waiting for turret align");
                }
            }
        }

        // Update turret
        double turretFF = 0;
        if (turretHome) {
            deposit.turretHome();
        } else {
            turretFF = updateTurret();
        }

        deposit.update(theta, turretFF);
        turretGlobalTheta = deposit.getTurretTheta() + theta - PI/2;

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

        // Update Tape Detector
        if (useTapeDetector) {
            double[] resetOdoCoords = tapeDetector.update(drivetrain.x, drivetrain.y, drivetrain.theta);
            resetOdo(drivetrain.x, resetOdoCoords[0], resetOdoCoords[1]);
        }

        profile(1);

        // Log Data
        if (loopCounter % loggerUpdatePeriod == 0) {
            logger.logData(curTime - startTime, x, y, theta, vx, vy, w, ax, ay, a,
                    turretGlobalTheta, slidesDist, depositTargetHeight, intakeSlidesHome, cycles, cycleTotal / cycles);
        }

        profile(2);

        // Dashboard Telemetry
        if (startVoltTooLow) {
            addPacket("0", "Starting Battery Voltage < 12.4!!!!");
        }
        addPacket("1 X", round(x));
        addPacket("2 Y", round(y));
        addPacket("3 Theta", round(theta));
        addPacket("4 Turret Theta", round(turretGlobalTheta));
        addPacket("5 Deposit Level", depositTargetHeight);
        addPacket("7 Automation Step", automationStep);
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

    public double updateTurret() {
        // Calculating the Coords of the Turret Center
        double[] turretCenter = new double[2];
        turretCenter[0] = x + Math.cos(theta) * Deposit.TURRET_Y_OFFSET;
        turretCenter[1] = y + Math.sin(theta) * Deposit.TURRET_Y_OFFSET;

        double turretFF;
        double goalX;
        double goalY;
        if (allianceHub && isRed) { // red
            goalX = redGoalCoords[0];
            goalY = redGoalCoords[1];
        } else if (allianceHub) { // blue
            goalX = blueGoalCoords[0];
            goalY = blueGoalCoords[1];
        } else { // neutral
            goalX = neutGoalCoords[0];
            goalY = neutGoalCoords[1];
        }
        goalX -= turretCenter[0];
        goalY -= turretCenter[1];

        lockTheta = PI + atan2(goalY, goalX);
        slidesDist = hypot(goalX, goalY);
        // calculates ff for turret control (w + atan dot)
        turretFF = w + (goalX * vy - vx * goalY) / (goalX * goalX + goalY * goalY);

        deposit.setTurretLockTheta(lockTheta);

        return turretFF;
    }

    // Set Depositor Controls
    public void depositHome() {
       deposit(Deposit.DepositHeight.HOME);
    }

    public void depositAllianceHub(Deposit.DepositHeight depositTargetHeight) {
        allianceHub = true;
        deposit(depositTargetHeight);
    }

    public void depositTrackSharedHub() {
        allianceHub = false;
        deposit(Deposit.DepositHeight.LOW);
    }

    public void deposit(Deposit.DepositHeight depositTargetHeight) {
        this.depositTargetHeight = depositTargetHeight;
        if (depositTargetHeight == Deposit.DepositHeight.LOW) {
            deposit.setDepositControls(Constants.DEPOSIT_ARM_LOW, slidesDist - Constants.ARM_DISTANCE_LOW);
        } else if (depositTargetHeight == Deposit.DepositHeight.MID) {
            deposit.setDepositControls(Constants.DEPOSIT_ARM_MID, slidesDist - Constants.ARM_DISTANCE_MID);
        } else if (depositTargetHeight == Deposit.DepositHeight.HIGH) {
            deposit.setDepositControls(Constants.DEPOSIT_ARM_HIGH, slidesDist - Constants.ARM_DISTANCE_HIGH);
        }  else { // Home
            deposit.setDepositControls(Constants.DEPOSIT_ARM_TRANSFER, slidesDist);
        }
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

    public boolean isAtPoseTurret(double turretTheta, double turretTolerance) {
        return abs(turretGlobalTheta - turretTheta) < turretTolerance;
    }

    public boolean isAtPoseTurret(double turretTheta) {
        return abs(turretGlobalTheta - turretTheta) < turretTolerance;
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
    public void automationStep(String step) {
        automationStep = step;
        log(automationStep);
    }

    @SuppressLint("DefaultLocale")
    public static double round(double num) {
        return Double.parseDouble(String.format("%.5f", num));
    }
}
