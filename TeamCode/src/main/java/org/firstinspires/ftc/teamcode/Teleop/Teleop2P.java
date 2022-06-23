package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Debug.Logger;
import org.firstinspires.ftc.teamcode.Localization.IMU;
import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@TeleOp(name = "0 Teleop 2P")
@Config
public class Teleop2P extends LinearOpMode {

    // Backup Starting Position
    public static double startX = 138;
    public static double startY = 81;
    public static double startTheta = PI / 2;

    public static boolean useAutoPos = true;
    public static boolean isRed = true;
    public static boolean fieldCentric = false;

    private Robot robot;
    private IMU imu;

    // Toggles
    private boolean duckActive = false;
    private boolean duckToggle = false;
    private boolean intakeToggle = false;
    private boolean defenseModeToggle = false;

    // Control Gains
    public static double xGain = 1;
    public static double yGain = 1;
    public static double wGain = 1;

    public static double xSlowGain = 0.5;
    public static double ySlowGain = 0.3;
    public static double wSlowGain = 0.4;

    public static boolean squaredControl = false;

    // Rumbles
    private boolean teleRumble1 = false;
    private boolean midTeleRumble = false;
    private boolean endgameRumble = false;

    private double starCarouselTime = 0;

    private double capUpOffset = 0;
    private double capDownOffset = 0;
    private boolean cappingDown = true;
//    private boolean fastHigh = true;
    public static double slidesOffsetAmount = 2.0;

    private boolean capToggle = false;
//    private boolean fastHighToggle = false;
    private boolean intakeApprovalToggle = false;
    private boolean midOffsetUpToggle = false;
    private boolean midOffsetDownToggle = false;
    private boolean oppSharedToggle = false;
    private boolean oppShared = false;

    /*
    Controller Buttons : *updated 6/23/22 2:12 PM*

    gamepad 1:

    Left Joystick: forward/back/right/left movement
    Right Joystick: turning
    Left Bumper (Top): extend deposit to alliance level 3 (press once); release freight (press again)
    Left Bumper (Bottom): extend deposit to alliance level 2 (press once); release freight (press again)
    Right Bumper (Top): intake without extending intake slides
    Right Bumper (Bottom): intake and extend intake slides
    Δ (Y): reverse intake
    O (B): extend deposit to shared hub (press once); release freight (press again)


    gamepad 2:

    Left Joystick: adjust the arm position for depositing in the shared hub
    Right Joystick: raise/lower arm to cap team shipping element (if robot is int the middle of the capping sequence - see response to question 7);
    offset the intake slides out or in
    Left Bumper (Top):
    Left Bumper (Bottom):
    Right Bumper (Top):
    Right Bumper (Bottom):
    DPad Up/Down: change the alliance hub tilt preset - the slides will go to a different height depending on if the alliance hub is balanced, tiled away, or tilted towards the drivers. DPad Up sets the preset to tilted away; DPad Down sets the preset to tilted towards the drivers.
    Δ (Y): advance the step of capping automation sequence
    X (A): when cycling freight into the shared hub - toggles between cycling from the alliance’s warehouse and cycling from the opposite alliance’s warehouse
    ☐(X): raise intake box (to traverse over barrier)
    */

    @Override
    public void runOpMode() throws InterruptedException {
        if (useAutoPos) {
            for (int i = 0; i<Logger.readPos().length; i++)
                addPacket("log " + i, Logger.readPos()[i]);
            robot = new Robot(this, true);
        } else {
            robot = new Robot(this, (isRed ? startX : 144 - startX), startY, startTheta, false, isRed, true);
        }

//        imu = new IMU(robot.theta, this);
        waitForStart();

//        robot.cycleHub = Robot.DepositTarget.fastHigh;

        ElapsedTime cycleTimer = new ElapsedTime();
        cycleTimer.reset();

        while (opModeIsActive()) {
            robot.depositApproval = gamepad1.left_trigger > .1 || gamepad1.b || gamepad1.left_bumper;

            boolean intakeApproval = gamepad1.right_trigger > .1 || gamepad1.right_bumper;
            if (robot.intakeApproval && !intakeApproval) robot.intakeApproval = false;
            else if (intakeApproval && !intakeApprovalToggle) {
                intakeApprovalToggle = true;
                robot.intakeApproval = true;
            } else if (!intakeApproval && intakeApprovalToggle) {
                intakeApprovalToggle = false;
            }

            robot.outtake = gamepad1.y;
            robot.intakeNoExtend = gamepad1.right_bumper;
            robot.intakeUp = gamepad1.x || gamepad2.x;

            if (robot.intakeRumble) gamepad1.rumble(500);
            if (robot.transferRumble) gamepad1.rumble(500);

//            if (!fastHighToggle && gamepad2.touchpad) {
//                fastHigh = !fastHigh;
//                fastHighToggle = true;
//            } else if (fastHighToggle && !gamepad2.touchpad) {
//                fastHighToggle = false;
//            }

            if (gamepad1.left_bumper) {
                robot.cycleHub = Robot.DepositTarget.mid;
            } else if (gamepad1.b) {
                if (oppShared) robot.cycleHub = Robot.DepositTarget.oppShared;
                else robot.cycleHub = Robot.DepositTarget.shared;
            } else if (gamepad1.left_trigger > .1) {
//                if (fastHigh) robot.cycleHub = Robot.DepositTarget.fastHigh;
                robot.cycleHub = Robot.DepositTarget.high;
            }

            if (gamepad2.right_bumper) robot.carousel.turnon();
            else  robot.carousel.turnoff();

            if (!capToggle && gamepad2.y) {
                capToggle = true;
                robot.advanceCapState();
            } else if (capToggle && !gamepad2.y) {
                capToggle = false;
            }

            if (gamepad2.start) robot.capMech.capNumber = 1;

            if (!oppSharedToggle && gamepad2.a) {
                oppSharedToggle = false;
                oppShared = !oppShared;
            } else if (oppSharedToggle && !gamepad2.a) {
                oppSharedToggle = true;
            }


            if (robot.depositState == 4 && robot.cycleHub == Robot.DepositTarget.high) {
                if (gamepad2.dpad_up) robot.deposit.highOffset += slidesOffsetAmount;
                if (gamepad2.dpad_down) robot.deposit.highOffset -= slidesOffsetAmount;
            } else if (robot.depositState == 4 && robot.cycleHub == Robot.DepositTarget.mid) {
                if (!midOffsetUpToggle && gamepad2.dpad_up) {
                    if (robot.deposit.midOffset < 1) robot.deposit.midOffset++;
                    midOffsetUpToggle = true;
                } else if (midOffsetUpToggle && !gamepad2.dpad_up) {
                    midOffsetUpToggle = false;
                }
                if (!midOffsetDownToggle && gamepad2.dpad_down) {
                    if (robot.deposit.midOffset > -1) robot.deposit.midOffset--;
                    midOffsetDownToggle = true;
                } else if (midOffsetDownToggle && !gamepad2.dpad_down) {
                    midOffsetDownToggle = false;
                }
                addPacket("mid offset", robot.deposit.midOffset);
            } else if (robot.depositState == 4 && (robot.cycleHub == Robot.DepositTarget.shared || robot.cycleHub == Robot.DepositTarget.oppShared)) {
                if (gamepad2.dpad_up) robot.deposit.sharedOffset -= 1;
                if (gamepad2.dpad_down) robot.deposit.sharedOffset += 1;
            } else {
//                if (gamepad2.dpad_up) robot.deposit.initialSlidesPos -= .4;
//                if (gamepad2.dpad_down) robot.deposit.initialSlidesPos += .4;
            }

            if (Math.abs(gamepad2.right_stick_y) > .1) {
                if (robot.capState == 4 || robot.capState == 5) {
                    robot.capMech.upOffset -= .007 * gamepad2.right_stick_y;
                } else if (robot.capState == 2 || robot.capState == 3) {
                    robot.capMech.downOffset -= .007 * gamepad2.right_stick_y;
                } else {
                    robot.intake.initialSlidesPos += .6 * gamepad2.right_stick_y;
                }
            }

            if (Math.abs(gamepad2.left_stick_y) > .1) {
                robot.deposit.sharedOffset += 0.005 * gamepad2.left_stick_y;
            }

            addPacket("initial slidesPos", robot.intake.initialSlidesPos);

            double xGain;
            double yGain;
            double wGain;

            if (robot.capState != 1 || robot.depositState == 4) {
                xGain = this.xSlowGain;
                yGain = this.ySlowGain;
                wGain = this.wSlowGain;
            } else {
                xGain = this.xGain;
                yGain = this.yGain;
                wGain = this.wGain;
            }

//            squaredControl = robot.capState == 4 || robot.capState == 5;

            robot.intakeExtendDist = Math.max(Constants.INTAKE_SLIDES_HOME_TICKS,Math.min(robot.intakeExtendDist + gamepad2.right_trigger - gamepad2.left_trigger, Constants.INTAKE_SLIDES_EXTEND_TICKS));

            // Drivetrain Controls
            // Field Centric Driving
            if (fieldCentric) {
                imu.updateHeading();
                double theta = imu.getTheta();
                double xControls = gamepad1.left_stick_x * xGain;
                double yControls = gamepad1.left_stick_y * yGain;
                robot.drivetrain.setControls(xControls * Math.sin(theta) + yControls * Math.cos(theta), xControls * Math.cos(theta) - yControls * Math.sin(theta), -gamepad1.right_stick_x * wGain);
            } else if (squaredControl) {
                robot.drivetrain.setControls(-Math.signum(gamepad1.left_stick_y) * gamepad1.left_stick_y * gamepad1.left_stick_y,
                        -Math.signum(gamepad1.left_stick_x) * gamepad1.left_stick_x * gamepad1.left_stick_x,
                        -Math.signum(gamepad1.right_stick_x) * gamepad1.right_stick_x * gamepad1.right_stick_x);
            } else {
                robot.drivetrain.setControls(yGain * -gamepad1.left_stick_y, xGain * -gamepad1.left_stick_x, wGain * -gamepad1.right_stick_x);
            }

            // Update Robot
            robot.update();

            // Telemetry
//            for (int i = 0; i < robot.cycles.size(); i++) {
//                telemetry.addData("cycle " + i, cycles.get(i));
//            }

            telemetry.addData("X", Robot.round(robot.x));
            telemetry.addData("Y", Robot.round(robot.y));
            telemetry.addData("Theta", Robot.round(robot.theta));
            telemetry.addData("Automation", robot.automationStep);
            telemetry.addData("Intake Full/Stall", robot.intakeFull + "/" + robot.intakeStalling);
            telemetry.addData("# Cycles", robot.cycles.size());
            telemetry.addData("Average Cycle Time", Robot.round(robot.cycleAvg) + "s");
            telemetry.update();
        }
//        if (robot.cycleList.size() > 0) {
//            Log.w("cycle-log", "Avg dropping longest: " + ((robot.cycleTotal - robot.longestCycle) / (robot.cycles - 1)) + "s");
//        }
        robot.stop();
    }
}
