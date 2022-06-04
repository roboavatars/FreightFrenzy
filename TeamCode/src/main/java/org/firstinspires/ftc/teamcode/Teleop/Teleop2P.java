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

@TeleOp(name = "2 Teleop 2P")
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
    private static double xyGain = 1;
    private static double wGain = 0.8;

    private static double xySlowGain = 0.4;
    private static double wSlowGain = 0.3;

    // Rumbles
    private boolean teleRumble1 = false;
    private boolean midTeleRumble = false;
    private boolean endgameRumble = false;

    private double starCarouselTime = 0;

    private double capUpOffset = 0;
    private double capDownOffset = 0;
    private boolean cappingDown = true;

    private boolean capToggle = false;
    private boolean midGoalToggle = false;
    private boolean intakeApprovalToggle = false;

    /*
    Controller Buttons : *updated 5/30/22 2:48 PM*
    gamepad 1:

    right trigger - intake
    left trigger - release
    left stick - xy movement
    right stick - turn

    gamepad 2:

    a - deposit
    y - capping mode
    left bumper - mid goal
    right bumper - carousel
    dpad up - decrease slide offset
    dpad down - increase slide offset
    right trigger - increase intake offset
    left trigger - decrease intake offset
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

        ElapsedTime cycleTimer = new ElapsedTime();
        cycleTimer.reset();

        while (opModeIsActive()) {
            robot.depositApproval = gamepad2.a;
            robot.releaseApproval = gamepad1.left_trigger > .1;
            if (robot.intakeApproval && gamepad1.right_trigger <= .1) robot.intakeApproval = false;
            else if (gamepad1.right_trigger > .1 && !intakeApprovalToggle) {
                intakeApprovalToggle = true;
                robot.intakeApproval = true;
            } else if (gamepad1.right_trigger <= .1 && intakeApprovalToggle) {
                intakeApprovalToggle = false;
            }
            robot.outtake = gamepad1.left_bumper;
            robot.intakeNoExtend = gamepad1.right_bumper;
            robot.intakeUp = gamepad2.b;


            if (gamepad2.dpad_right) {
                robot.cycleHub = Robot.DepositTarget.mid;
            } else if (gamepad2.dpad_left) {
                robot.cycleHub = Robot.DepositTarget.low;
            } else if (gamepad2.left_bumper) {
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

            if (robot.capping) {
                if (robot.capState == 1) {
                    if (gamepad2.dpad_up)
                        robot.capOffset -= .01;
                    if (gamepad2.dpad_down)
                        robot.capOffset += .01;
                } else if (robot.capState == 2)
                    if (gamepad2.dpad_up)
                        robot.capOffset -= .01;
                    if (gamepad2.dpad_down)
                        robot.capOffset += .01;
            } else {
                if (gamepad2.dpad_up) robot.deposit.initialSlidesPos -= .4;
                if (gamepad2.dpad_down) robot.deposit.initialSlidesPos += .4;
            }

            double xyGain;
            double wGain;

            if (robot.capping || robot.depositState == 4) {
                xyGain = this.xySlowGain;
                wGain = this.wSlowGain;
            } else {
                xyGain = this.xyGain;
                wGain = this.wGain;
            }

            robot.intakeExtendDist = Math.max(Constants.INTAKE_SLIDES_HOME_TICKS,Math.min(robot.intakeExtendDist + gamepad2.right_trigger - gamepad2.left_trigger, Constants.INTAKE_SLIDES_EXTEND_TICKS));

            // Drivetrain Controls
            // Field Centric Driving
            if (fieldCentric) {
                imu.updateHeading();
                double theta = imu.getTheta();
                double xControls = gamepad1.left_stick_x * xyGain;
                double yControls = gamepad1.left_stick_y * xyGain;
                robot.drivetrain.setControls(xControls * Math.sin(theta) + yControls * Math.cos(theta), xControls * Math.cos(theta) - yControls * Math.sin(theta), -gamepad1.right_stick_x * wGain);
            } else {
                robot.drivetrain.setControls(xyGain * -gamepad1.left_stick_y, xyGain * -gamepad1.left_stick_x, wGain * -gamepad1.right_stick_x);
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
