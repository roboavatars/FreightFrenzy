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

import org.firstinspires.ftc.teamcode.Debug.Logger;
import org.firstinspires.ftc.teamcode.Pathing.Pose;
import org.firstinspires.ftc.teamcode.Pathing.Target;

import java.util.ArrayList;
import java.util.List;

@SuppressWarnings("FieldCanBeLocal")
@Config
public class Robot {

    public static boolean turretEnabled = false;
    public double capOffset = 0;

    // Robot Classes
    public Drivetrain drivetrain;
    public Intake intake;
    public Carousel carousel;
    // public TapeDetector tapeDetector;
    public Logger logger;
    public Deposit deposit;

    //    private final ElapsedTime profiler;
    private final List<LynxModule> allHubs;
    private final VoltageSensor battery;
    private double voltage;
    private final double startVoltage;

    // Class Constants
    private final int loggerUpdatePeriod = 2;
    private final int sensorUpdatePeriod = 10;
    private final int stallUpdatePeriod = 5;
    private final int voltageUpdatePeriod = 1000;
    private final double xyTolerance = 1;
    private final double thetaTolerance = PI / 35;
    public final static double[] cameraRelativeToRobot = new double[]{1, 3};

    // State Variables
    private final boolean isAuto;
    public final boolean isRed;
    private boolean firstLoop = true;
    private int loopCounter = 0;
    public String automationStep = "n/a";
    public String antiStallStep = "n/a";

    public boolean intakeFull;
    public boolean intakeStalling;
    public double turretGlobalTheta;
    public boolean reverseIntake = false;


    // Automation Variables
    public boolean depositApproval = false;
    public boolean releaseApproval = false;
    public boolean transferVerify = false;
    public boolean intakeApproval = false;
    public boolean midGoal = false;
    public boolean intakeTransfer = false;
    public boolean slidesInCommand = false;
    public boolean depositingFreight = false;
    public boolean armExtendCommands = false;
    public boolean intakeRev = false;
    private final double intakeFlipTime = -1;
    private final double armOpenTime = -1;
    private final double slidesAtPosTime = -1;
    private final double extendTime = -1;
    public boolean autoNoTurret = false;
    //    public boolean carouselAuto = false;
    public boolean depositEnabled = true;

    public int depositState = 0;
    public int sharedState = 0;
    public boolean capping = false;
    public int capState = 0;
    private double transferStart;
    private double startExtendTime;
    private double sharedDepositStart;
    private double sharedRetractStart;
    private double depositStart;
    private double depositStartRetract;
    private double intakeRetractStart;
    public int intakeState = 1;
    public static double cubeTransferThreshold = 750;
    public static double ballTransferThreshold = 1000;
    public static double turretDepositThreshold = 1000;
    public static double turretHomeThreshold = 1000;
    public static double releaseThreshold = 500;
    public static double intakeFlipThreshold = 500;
    public static double armFlipThreshold = 1000;
    public static double armReturnThreshold = 1000;
    public String element;
    public static double intakeExtendDist = Constants.INTAKE_SLIDES_EXTEND_TICKS; //(Constants.INTAKE_SLIDES_HOME_TICKS + Constants.INTAKE_SLIDES_EXTEND_TICKS)/2;
    public boolean rumble = false;

    // Cycle Tracker
    public ArrayList<Double> cycles = new ArrayList<>();
    public double cycleAvg = 0;
    public double lastCycleTime;
    public double longestCycle = 0;
    public boolean autoFirstCycle = false;
    public static boolean shared = false;

    // Time and Delay Variables
    public double curTime;
    public static int flipUpThreshold = 700;
//    public static int hubTipThreshold = 300;



    public static int convergeThreshold = 1500;

    public double stallStartTime = -1;
    public static int stallThreshold = 500;
    public double automationStepTime;
    public double depositTime = 0;
    public static double startIntakingAutoY = 106;
    public static double extendDepositAutoY = 95;

    // Motion Variables
    public double x, y, theta, vx, vy, w;
    private double prevX, prevY, prevTheta, prevVx, prevVy, prevW, prevTime, ax, ay, a;
    public double startTime;

    // OpMode Stuff
    private final LinearOpMode op;

    public enum DepositTarget {
        high,
        mid,
        low,
        cap
    }

    public DepositTarget cycleHub;

    // Constructor
    public Robot(LinearOpMode op, double x, double y, double theta, boolean isAuto, boolean isRed) {
        this(op, x, y, theta, isAuto, isRed, 0,0, false);
    }
    public Robot(LinearOpMode op, double x, double y, double theta, boolean isAuto, boolean isRed, boolean startLogger) {
        this(op, x, y, theta, isAuto, isRed, 0,0, startLogger);
    }

    public Robot(LinearOpMode op, boolean startLogger) {
        this(op, Logger.readPos()[1], Logger.readPos()[2], Logger.readPos()[3], false, Logger.readPos()[0] == 1, (int) Logger.readPos()[6], (int) Logger.readPos()[7], startLogger);
    }

    public Robot(LinearOpMode op, double x, double y, double theta, boolean isAuto, boolean isRed, int depositSlidesPos, int intakeSlidesPos, boolean startLogger) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.op = op;
        this.isAuto = isAuto;
        this.isRed = isRed;

        // init subsystems
        drivetrain = new Drivetrain(op, x, y, theta, isAuto);
        carousel = new Carousel(op, isRed);
        logger = new Logger();

        deposit = new Deposit(op, isAuto, depositSlidesPos);
        intake = new Intake(op, isAuto, intakeSlidesPos);

        addPacket("depositSlides", depositSlidesPos);
        addPacket("intakeSlides", intakeSlidesPos);


        //        tapeDetector = new TapeDetector(op);

        // set up bulk read
        allHubs = op.hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // low voltage warning
        battery = op.hardwareMap.voltageSensor.iterator().next();
        voltage = round(battery.getVoltage());
        log("Battery Voltage: " + voltage + "v");
        startVoltage = voltage;
//        profiler = new ElapsedTime();

        // Initial Dashboard Drawings
//        if (intake.getDistance() > 1000) {
//            addPacket("0 DISTANCE SENSOR", "> 1000!!!");
//            op.telemetry.addData("0 DISTANCE SENSOR", "> 1000!!!");
//            op.telemetry.update();
//        }

        drawField();
        drawRobot(this);
        sendPacket();

        cycleHub = DepositTarget.high;

        if(startLogger) logger.startLogging(isAuto, isRed);

    }

    // Stop logger
    public void stop() {
        Log.w("cycle-log", "All cycles:");
        for (int i = 0; i < cycles.size(); i++)
            Log.w("cycle-log", "Cycle " + (i + 1) + ": " + cycles.get(i) + "s");
        Log.w("cycle-log", "Avg cycle Time: " + round(cycleAvg) + "s");
        drivetrain.stop();
        logger.stopLogging();
    }

    // Reset Odometry
    public void resetOdo(double x, double y, double theta) {
        drivetrain.resetOdo(x, y, theta);
    }

    public void update() {
//        profiler.reset();
        curTime = System.currentTimeMillis();

        // Don't check states every loop
        if (loopCounter % sensorUpdatePeriod == 0 && intakeState != 1) {
            intakeFull = intake.isFull();
            element = intake.getElement();
        }
        addPacket("intake current", intake.getCurrent());
//        profile(1);
        if (loopCounter % stallUpdatePeriod == 0 && intakeState != 1) {
            intakeStalling = intake.checkIfStalling();;
        }
//        profile(2);
        if (loopCounter % voltageUpdatePeriod == 0) {
            voltage = round(battery.getVoltage());
        }

        loopCounter++;

//        profile(3);

        // Track time after start
        if (firstLoop) {
            startTime = curTime;
            lastCycleTime = curTime;
        }

//        profile(8);

        // Update Position
        drivetrain.updatePose();

//        profile(9);

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

        /*// Update Tape Detector
        if (useTapeDetector) {
            double[] resetOdoCoords = tapeDetector.update(drivetrain.x, drivetrain.y, drivetrain.theta);
            resetOdo(drivetrain.x, resetOdoCoords[0], resetOdoCoords[1]);
        }*/

        // Log Data
        if (loopCounter % loggerUpdatePeriod == 0) {
            logger.logData(curTime - startTime, x, y, theta, vx, vy, w, ax, ay, a,
                    deposit.getSlidesPos(), cycleHub, intake.getSlidesPos(), intakeState, depositState,
                    cycles.size(), cycleAvg);
        }

//        profile(10);

        // Dashboard Telemetry
        addPacket("1 Voltage", startVoltage + " -> " + voltage);
        addPacket("2 X", round(x));
        addPacket("3 Y", round(y));
        addPacket("4 Theta", round(theta));
        addPacket("5 Turret Theta", round(turretGlobalTheta));
        addPacket("7 Automation Step", automationStep + "; " + antiStallStep);
        addPacket("7 Automation", intakeTransfer + "; " + depositingFreight);
        addPacket("8 Intake Full", intakeFull);
        addPacket("81 Intake Sensor Distance", intake.getDistance());
        addPacket("9 Intake Stalling", intakeStalling);
//        addPacket("Carousel Velocity", carousel.getVelocity());
        addPacket("91 Run Time", (curTime - startTime) / 1000);
        addPacket("92 Update Frequency (Hz)", round(1 / timeDiff));
        addPacket("pod zeroes", drivetrain.zero1 + " " + drivetrain.zero2 + " " + drivetrain.zero3);
        addPacket("intake slides", intake.getSlidesPos());
        addPacket("deposit slides", deposit.getSlidesPos());

        if (!isAuto) {
            addPacket("Cycle Time", (curTime - lastCycleTime) / 1000);
            addPacket("Average Cycle Time", round(cycleAvg));
            addPacket("Cycles", cycles.size());
        }

        // Dashboard Drawings
        drawField();
        drawRobot(this);
        sendPacket();

//        profile(11);

        // Clear bulk cache/ print current
//        List<Double> hubCurrents = new ArrayList<>();
        for (LynxModule hub : allHubs) {
//            hubCurrents.add(hub.getCurrent(CurrentUnit.AMPS));
            hub.clearBulkCache();
        }
//        Log.w("hub current draw", "1: " + hubCurrents.get(1) + "; 2: " + hubCurrents.get(hubCurrents.size() - 1));
//        addPacket("hub current draw", "1: " + hubCurrents.get(1) + "; 2: " + hubCurrents.get(hubCurrents.size() - 1));

//        profile(12);

        firstLoop = false;
        if (intakeApproval && !isAuto) {
            intakeState = 2;
            depositState = 1;
        }

        boolean waitForIntakeFlip = false;
        rumble = false;
        switch (intakeState) {
            case 1: //intake home
                intake.flipDown();
                intake.off();
                if (isAuto && intakeApproval && y >= startIntakingAutoY) intakeState++;
                break;
            case 2: //intake freight
                if (isAuto) intake.extend();
                else intake.setSlidesPosition((int) Math.round(intakeExtendDist));
                intake.flipDown();
                if (intakeFull || (!isAuto && !intakeApproval)) {
                    intakeState++;
                    intakeRetractStart = System.currentTimeMillis();
                    if (element == "ball" || midGoal) cycleHub = DepositTarget.mid;
                    else cycleHub = DepositTarget.high;
                } else if (intakeFull && !isAuto && intakeApproval) {
                    rumble = true;
                }

                //anti-stall
                if (isAuto) {
                    if (!intakeStalling) {
                        stallStartTime = -1;
                        intake.on();
                        antiStallStep = "Intake On";
                        automationStep(antiStallStep);
                    } else if (stallStartTime == -1) {
                        stallStartTime = curTime;
                        antiStallStep = "Jam Detected";
                        automationStep(antiStallStep);
                    } else if (curTime - stallStartTime > stallThreshold) {
                        intake.reverse();
                        antiStallStep = "Reverse Intake";
                        automationStep(antiStallStep);
                    }
                } else {
                    intake.on();
                }
                break;
            case 3: //wait for flip servo and intake slides
                intake.home();
                intake.setPower(.5);
                intake.flipUp();
                if (intake.slidesIsHome() && System.currentTimeMillis() - intakeRetractStart > intakeFlipThreshold)
                    intakeState++;
                break;
            case 4: //wait for deposit to retract
                if (depositState == 1) {
                    intakeState++;
                    transferStart = System.currentTimeMillis();
                }
                break;
            case 5: //transfer
                intake.reverse();
                if ((!isAuto && depositApproval) || (System.currentTimeMillis() - transferStart > (cycleHub == DepositTarget.mid ? ballTransferThreshold : cubeTransferThreshold))) {
                    intakeState = 1;
                    depositState = 2;
                    sharedState = 2;
                }
                break;
        }
        intake.updateSlides(ay);

        //deposit states
//        deposit.turretHome();
        if (capping) {
            switch (capState) {
                case 1:
                    deposit.retractSlides();
                    deposit.setArmControls(Constants.ARM_CAP_DOWN_POS - capOffset);
                    deposit.setServoPos(Constants.DEPOSIT_CAP_POS);
                    break;
                case 2:
                    deposit.extendSlides(cycleHub);
                    deposit.setArmControls(Constants.ARM_CAP_UP_POS - capOffset);
                    deposit.setServoPos(Constants.DEPOSIT_CAP_POS);
                    break;
            }
        } else {
            switch (depositState) {
                case 1: //deposit home
                    deposit.retractSlides();
                    deposit.armHome();
                    deposit.open();
                    break;
                case 2: //once transfer done, hold freight
                    deposit.hold();
                    if ((isAuto && y <= extendDepositAutoY) || depositApproval) depositState++;
                    break;
                case 3: //extend deposit
                    deposit.extendSlides(cycleHub);
                    deposit.armOut();
                    deposit.hold();
                    startExtendTime = curTime;
                    if (isAuto || !depositApproval) depositState++;
                    break;
                case 4: //wait for driver approval for release
                    if ((!isAuto || (deposit.slidesAtPos() && curTime - startExtendTime > armFlipThreshold)) && (depositApproval || releaseApproval)) {
                        depositState++;
                        depositStart = System.currentTimeMillis();
                    }
                    break;
                case 5: //release & wait for freight to drop
                    deposit.release();
                    if (System.currentTimeMillis() - depositStart > releaseThreshold) {
                        depositState++;
                    }
                    break;
                case 6:
                    markCycle();
                    depositStartRetract = curTime;
                    depositState++;
                    break;
                case 7:
                    deposit.retractSlides();
                    deposit.armHome();
                    if (curTime - depositStartRetract > armReturnThreshold) {
                        depositState = 1;
                    }
                    break;
            }
        }
        deposit.updateSlides();
        addPacket("deposit state", depositState);
        addPacket("intake state", intakeState);
        addPacket("element", element == "ball" ? 0 : 1);

    }

//    public void setCycleHub(DepositTarget cycleHub) {
//        this.cycleHub = cycleHub;
//        depositState = 1;
//
//        if (cycleHub == DepositTarget.high || cycleHub == DepositTarget.neutral) {
//            turret.setHome();
//            arm.setHome();
//        }
//    }


    public void advanceCapState() {
        capping = true;
        capOffset = 0;
        if (capState < 2)
            capState++;
        else {
            capState = 0;
            depositState = 1;
            capping = false;
        }
    }

    // Keep track of cycles
    public void markCycle() {
        double cycleTime = (curTime - lastCycleTime) / 1000;
        cycles.add(cycleTime);
        cycleAvg = cycles.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);

        Log.w("cycle-log", "Cycle " + cycles.size() + ": " + cycleTime + "s");
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
        //Log.w("profiler", num + ": " + profiler.milliseconds());
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
