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
import org.firstinspires.ftc.teamcode.Pathing.Pose;
import org.firstinspires.ftc.teamcode.Pathing.Target;

import java.util.ArrayList;
import java.util.List;

@SuppressWarnings("FieldCanBeLocal")
@Config
public class Robot {

    public static boolean turretEnabled = false;

    // Robot Classes
    public Drivetrain drivetrain;
    public Intake intake;
    public Carousel carousel;
    // public TapeDetector tapeDetector;
    public Logger logger;
    public Deposit deposit;
    public CapMech capMech;

    private final ElapsedTime profiler;
    private final List<LynxModule> allHubs;
    private final VoltageSensor battery;
    private double voltage;
    private final double startVoltage;

    // Class Constants
    private final int loggerUpdatePeriod = 2;
    private final int sensorUpdatePeriod = 5;
    private final int stallUpdatePeriod = 5;
    private final int voltageUpdatePeriod = 1000;
    private final int imuUpdatePeriod = 10;
    private final double xyTolerance = 1;
    private final double thetaTolerance = PI / 35;
    public final static double[] cameraRelativeToRobot = new double[]{1, 3};

    // State Variables
    private final boolean isAuto;
    public final boolean isRed;
    public boolean carouselAuto = false;
    private boolean firstLoop = true;
    private int loopCounter = 0;
    public String automationStep = "n/a";
    public String antiStallStep = "n/a";

    public boolean intakeFull;
    public boolean intakeTransferred;
    public boolean intakeStalling;
    public double intakeSensorDist;


    // Automation Variables
    public boolean depositApproval = false;
    public boolean releaseApproval = false;
    public boolean transferVerify = false;
    public boolean intakeApproval = false;
    public boolean transferOverride = false;
    public boolean outtake = false;
    public boolean intakeNoExtend = false;
    public boolean intakeUp = false;
    public boolean capDown = false;
    public boolean midGoal = false;
    public boolean intakeTransfer = false;
    public boolean slidesInCommand = false;
    public boolean depositingFreight = false;
    public boolean intakeRev = false;
    private final double intakeFlipTime = -1;
    private final double slidesAtPosTime = -1;
    private final double extendTime = -1;
    public boolean autoNoTurret = false;
    //    public boolean carouselAuto = false;
    public boolean depositEnabled = true;

    public int depositState = -1;
    public int sharedState = 0;
    public int capState = 1;
    private double transferStart;
    private double startExtendTime;
    private double sharedDepositStart;
    private double sharedRetractStart;
    private double depositStart;
    private double depositStartRetract;
    private double intakeRetractStart;
    private double freightDetectedTime;
    private double clampStart = -Double.MAX_VALUE;
    public int intakeState = 1;
    public boolean intakeEnabled = true;
    public static double teleTransferThreshold = 750;
    public static double autoTransferThreshold = 1000;
    public static double turretDepositThreshold = 1000;
    public static double turretHomeThreshold = 1000;
    public static double teleReleaseThreshold = 250;
    public static double autoReleaseThreshold = 250;
    public static double duckReleaseThreshold = 500;
    public static double teleIntakeFlipThreshold = 400;
    public static double autoIntakeFlipThreshold = 800;
    public static double duckIntakeFlipThreshold = 1500;
    public static double retractDepositThreshold = 300;
    public static double clampThreshold = 200;
    public static double waitClampThreshold = 150;
    public static double duckTransferThreshold = 150;


    //    public String element;
    public double intakeExtendDist = Constants.INTAKE_SLIDES_EXTEND_TICKS; //(Constants.INTAKE_SLIDES_HOME_TICKS + Constants.INTAKE_SLIDES_EXTEND_TICKS)/2;
    public boolean rumble = false;

    // Cycle Tracker
    public ArrayList<Double> cycles = new ArrayList<>();
    public ArrayList<Double> intakeDepositTimes = new ArrayList<>();
    private double intakeDepositTimesAverage = 0;
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
    public static int stallThreshold = 750;
    public double automationStepTime;
    public double depositTime = 0;
    public static double startIntakingRedAutoY = 106;
    public static double startIntakingBlueAutoY = 106;
    public static double extendDepositAutoY = 95;

    // Motion Variables
    public double x, y, theta, vx, vy, w;
    private double prevX, prevY, prevTheta, prevVx, prevVy, prevW, prevTime, ax, ay, a;
    public double startTime;

    public double IMUay;

    // OpMode Stuff
    private final LinearOpMode op;

    public enum DepositTarget {
        high,
        fastHigh,
        mid,
        low,
        shared,
        cap,
        duck
    }

    public DepositTarget cycleHub;

    // Constructor
    public Robot(LinearOpMode op, double x, double y, double theta, boolean isAuto, boolean isRed) {
        this(op, x, y, theta, isAuto, isRed, false, false, true);
    }

    public Robot(LinearOpMode op, double x, double y, double theta, boolean isAuto, boolean isRed, boolean startLogger, boolean carouselAuto) {
        this(op, x, y, theta, isAuto, isRed, startLogger, carouselAuto, true);
    }

    public Robot(LinearOpMode op, double x, double y, double theta, boolean isAuto, boolean isRed, boolean startLogger) {
        this(op, x, y, theta, isAuto, isRed, startLogger, false, true);
    }

    public Robot(LinearOpMode op, boolean startLogger) {
        this(op, Logger.readPos()[1], Logger.readPos()[2], Logger.readPos()[3], false, Logger.readPos()[0] == 1, startLogger, false, false);
    }

    public Robot(LinearOpMode op, double x, double y, double theta, boolean isAuto, boolean isRed, boolean startLogger, boolean carouselAuto, boolean resetEncoders) {
        profiler = new ElapsedTime();
        profiler.reset();

        this.x = x;
        this.y = y;
        this.theta = theta;
        this.op = op;
        this.isAuto = isAuto;
        this.isRed = isRed;
        this.carouselAuto = carouselAuto;

        // init subsystems
        drivetrain = new Drivetrain(op, x, y, theta, isAuto);
        profile(1);
        carousel = new Carousel(op, isAuto, isRed);
        profile(2);
        logger = new Logger();
        profile(3);
        deposit = new Deposit(op, isAuto, (isAuto || resetEncoders));
        profile(4);
        intake = new Intake(op, isAuto, (isAuto || resetEncoders));
        profile(5);
        capMech = new CapMech(op, isAuto);
        profile(6);
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

        if (startLogger) logger.startLogging(isAuto, isRed);

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
        curTime = System.currentTimeMillis();

        // Don't check states every loop
        if (loopCounter % sensorUpdatePeriod == 0) {
            intakeFull = intake.isFull();
            intakeTransferred = intake.transferred();
            intakeSensorDist = intake.getDistance();
        }
        addPacket("intake current", intake.getCurrent());
//        profile(1);
        if (loopCounter % stallUpdatePeriod == 0 && intakeState == 2 || intakeState == 3) {
            intakeStalling = intake.checkIfStalling();
        }
//        profile(2);
        if (loopCounter % voltageUpdatePeriod == 0) {
            voltage = round(battery.getVoltage());
        }

        if (loopCounter % imuUpdatePeriod == 0) {
//            IMUay = -drivetrain.getAccel().yAccel;
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
        addPacket("7 Automation Step", automationStep + "; " + antiStallStep);
        addPacket("7 Automation", intakeTransfer + "; " + depositingFreight);
        addPacket("8 Intake Full", intakeFull);
//        addPacket("81 Intake Sensor Distance", intake.getDistance());
        addPacket("9 Intake Stalling", intakeStalling);
//        addPacket("Carousel Velocity", carousel.getVelocity());
        addPacket("91 Run Time", (curTime - startTime) / 1000);
        addPacket("92 Update Frequency (Hz)", round(1 / timeDiff));
        addPacket("pod zeroes", drivetrain.zero1 + " " + drivetrain.zero2 + " " + drivetrain.zero3);
        addPacket("intake slides", intake.getSlidesPos());
        addPacket("deposit slides", deposit.getSlidesPos());
        addPacket("Intake Sensor", intakeSensorDist);

        if (!isAuto) {
            addPacket("z0 Current Time", (curTime - lastCycleTime) / 1000);
            addPacket("z1 Last Time", cycles.size() > 0 ? cycles.get(cycles.size()-1) : 0);
            addPacket("z2 Average Cycle Time", round(cycleAvg));
            addPacket("z3 Cycles", cycles.size());

            addPacket("z4 Intake to deposit time", intakeDepositTimes.size() > 0 ? intakeDepositTimes.get(intakeDepositTimes.size()-1) : 0);
            addPacket("z5 Intake to deposit average", intakeDepositTimesAverage);
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
        if ((intakeApproval || outtake || intakeNoExtend) && !isAuto) {
            intakeState = 2;
            depositState = 1;
        }

        boolean waitForIntakeFlip = false;
        rumble = false;
        if (!intakeEnabled) intakeState = 7;
        switch (intakeState) {
            case 1: //intake home
                if (!intakeUp && !(!isAuto && cycleHub == DepositTarget.shared) && capState == 1)
                    intake.flipDown();
                else intake.flipUp();
                intake.off();
                if (isAuto && intakeApproval && (y >= (isRed ?  startIntakingRedAutoY : startIntakingBlueAutoY) || carouselAuto)) {
                    intakeState++;
                    intakeFull = intake.isFull();
                    intakeStalling = intake.checkIfStalling();
                }
                break;
            case 2: //intake freight
//                if (isAuto) intake.extend();
                if (!intakeNoExtend) intake.setSlidesPosition((int) Math.round(intakeExtendDist));
                else intake.home();
                intake.flipDown(cycleHub);

                //anti-stall
                if (isAuto && !carouselAuto) {
                    if (!intakeStalling || intakeFull) {
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
                    if (!outtake) intake.on();
                    else intake.reverse();
                }

                boolean intakeFull;
                if (!this.intakeFull) {
                    freightDetectedTime = System.currentTimeMillis();
                    intakeFull = false;
                } else intakeFull = System.currentTimeMillis() - freightDetectedTime > (isAuto ? Constants.INTAKE_TIME_THRESHOLD_AUTO : Constants.INTAKE_TIME_THRESHOLD_TELE);

                if ((isAuto && intakeFull) || (!isAuto && !intakeApproval && !outtake) || transferOverride) {
                    intake.off();
                    intakeState++;
                    intakeRetractStart = System.currentTimeMillis();
//                    if (element == "ball" || midGoal) cycleHub = DepositTarget.mid;
//                    else cycleHub = DepositTarget.high;
                    if (isAuto && !carouselAuto) cycleHub = DepositTarget.high;
                }
                if (intakeFull && !isAuto && intakeApproval) {
//                    intakeApproval = false;
                    rumble = true;
                }
                break;
            case 3: //wait for flip servo and intake slides
                intake.home(cycleHub);
                intake.setPower(Constants.INTAKE_RETRACT_POWER);
                intake.flipUp(cycleHub);
                if (intake.slidesIsHome() && ((System.currentTimeMillis() - intakeRetractStart) > ((isAuto) ? autoIntakeFlipThreshold : teleIntakeFlipThreshold)))
                    intakeState++;
                break;
            case 4: //wait for deposit to retract
                if (depositState == 1 && deposit.slidesisHome()) {
                    intakeState++;
                    intakeTransferred = intake.transferred();
                    transferStart = System.currentTimeMillis();
                }
                break;
            case 5: //transfer
                intake.setPower(carouselAuto ? Constants.INTAKE_DUCK_TRANSFER_POWER : Constants.INTAKE_TRANSFER_POWER);
                if ((!isAuto && depositApproval) || (!carouselAuto && intakeTransferred) || (carouselAuto && curTime - transferStart > duckTransferThreshold)) {//(System.currentTimeMillis() - transferStart > (isAuto ? autoTransferThreshold : teleTransferThreshold))) {
                    intakeState++;
                    clampStart = System.currentTimeMillis();
                    depositState = 2;
                    sharedState = 2;
                }
                break;
            case 6: //wait for deposit to clamp down on freight
//                intake.off();
                if (System.currentTimeMillis() - clampStart > clampThreshold) intakeState = 1;
                break;
            case 7: //intake off toggle
                intake.off();
                intake.home();
                if (intakeUp) intake.flipUp();
                else intake.flipDown();
                break;
        }
        intake.updateSlides(IMUay);

        //deposit states
//        deposit.turretHome();
        if (!depositEnabled) depositState = 8;
        switch (depositState) {
            case 0: //auto delay
                deposit.retractSlides();
                deposit.setArmControls(Constants.ARM_INIT_POS);
                deposit.hold();
                break;
            case 1: //deposit home
                deposit.retractSlides();
                deposit.armHome();
                deposit.open();
                deposit.sharedOffset = 0;
                break;
            case 2: //once transfer done, hold freight
                if (System.currentTimeMillis() - clampStart > waitClampThreshold) deposit.hold(cycleHub);
                if (cycleHub == DepositTarget.duck) deposit.setArmControls(Constants.ARM_DUCK_HOME_POS);
                if (((isAuto && !carouselAuto && y <= extendDepositAutoY) || depositApproval) && (System.currentTimeMillis() - clampStart > waitClampThreshold))
                    depositState++;
                break;
            case 3: //extend deposit
                deposit.extendSlides(cycleHub);
                deposit.armOut(cycleHub);
                deposit.hold(cycleHub);
                startExtendTime = curTime;
                if (isAuto || !depositApproval) depositState++;
                break;
            case 4: //wait for driver approval for release
                deposit.extendSlides(cycleHub);
                deposit.armOut(cycleHub);
                if ((!isAuto || (deposit.slidesAtPos() && deposit.isArmAtPos())) && ((!carouselAuto && depositApproval) || releaseApproval)) {
                    depositState++;
                    depositStart = System.currentTimeMillis();
                }
                break;
            case 5: //release & wait for freight to drop
                deposit.release(cycleHub);
                if (cycleHub == DepositTarget.duck) deposit.armOut();
                if (System.currentTimeMillis() - depositStart > (isAuto ? (carouselAuto ? duckReleaseThreshold : autoReleaseThreshold) : teleReleaseThreshold)) {
                    depositState++;
                }
                break;
            case 6:
                markCycle();
                depositStartRetract = curTime;
                depositState++;
                break;
            case 7:
                deposit.armHome();
                if (curTime - depositStartRetract > retractDepositThreshold) deposit.retractSlides();
                if (deposit.isArmHome()) {
                    depositState = 1;
                }
                break;
            case 8:
                deposit.retractSlides();
                deposit.armHome();
        }
        deposit.update(cycleHub);
        addPacket("deposit state", depositState);
        addPacket("intake state", intakeState);

        switch (capState) {
            case 1: //arm up
                if (capDown) capMech.init();
                else capMech.home();
                capMech.close();
                break;
            case 2: //picking up tse
                capMech.down();
                capMech.open();
                break;
            case 3: //hold cap
                capMech.down();
                capMech.close();
                break;
            case 4: //capping
                capMech.up();
                capMech.close();
                break;
            case 5: //release cap
                capMech.up();
                capMech.open();
        }
    }

    public void advanceCapState() {
        if (capState != 4 && capState != 2) capMech.upOffset = capMech.downOffset = 0;
        if (capState < 5)
            capState++;
        else {
            capMech.capNumber = 2;
            capState = 0;
        }
    }

    // Keep track of cycles
    public void markCycle() {
        double cycleTime = (curTime - lastCycleTime) / 1000;
        cycles.add(cycleTime);
        cycleAvg = cycles.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);

        intakeDepositTimes.add((System.currentTimeMillis() - intakeRetractStart)/1000);
        intakeDepositTimesAverage = intakeDepositTimes.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);

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
        Log.w("profiler", num + ": " + profiler.milliseconds());
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
