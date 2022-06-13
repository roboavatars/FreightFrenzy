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

@TeleOp(name = "1 Teleop 1P")
@Config
public class Teleop1P extends LinearOpMode {

    // Backup Starting Position
    public static double startX = 138;
    public static double startY = 81;
    public static double startTheta = PI / 2;

    public static boolean useAutoPos = false;
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
    public static double xyGain = 1;
    public static double wGain = 1;

    public static double xySlowGain = 0.3;
    public static double wSlowGain = 0.4;

    // Rumbles
    private boolean teleRumble1 = false;
    private boolean midTeleRumble = false;
    private boolean endgameRumble = false;

    private double starCarouselTime = 0;

    private double capUpOffset = 0;
    private double capDownOffset = 0;
    private boolean cappingDown = true;

    private boolean intakeApprovalToggle = false;

    /*
    Controller Button Mappings: *updated 5/30/22 2:55 PM*

    gamepad 1:
    right trigger - intake
    left trigger - deposit
    left stick - move
    right stick - turn
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
        robot.intakeExtendDist = Constants.INTAKE_SLIDES_EXTEND_TICKS;
        waitForStart();

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

            if (robot.rumble) gamepad1.rumble(500);

            if (robot.capState == 4 || robot.capState == 5) {
                if (gamepad1.dpad_up) robot.capArm.upOffset += .007;
                if (gamepad1.dpad_down) robot.capArm.upOffset -= .007;
            } else if (robot.depositState == 4 && robot.cycleHub == Robot.DepositTarget.high) {
                if (gamepad1.dpad_up) robot.deposit.highOffset += 1;
                if (gamepad1.dpad_down) robot.deposit.highOffset -= 1;
            } else if (robot.depositState == 4 && robot.cycleHub == Robot.DepositTarget.mid) {
                if (gamepad1.dpad_up) robot.deposit.midOffset += 1;
                if (gamepad1.dpad_down) robot.deposit.midOffset -= 1;
            } else if (robot.depositState == 4 && robot.cycleHub == Robot.DepositTarget.shared) {
                if (gamepad1.dpad_up) robot.deposit.sharedOffset -= 0.003;
                if (gamepad1.dpad_down) robot.deposit.sharedOffset += 0.003;
            } else {
                if (gamepad1.dpad_up) robot.deposit.initialSlidesPos -= .4;
                if (gamepad1.dpad_down) robot.deposit.initialSlidesPos += .4;
            }

            if (gamepad1.dpad_left) robot.intake.initialSlidesPos += .4;
            if (gamepad1.dpad_right) robot.intake.initialSlidesPos -= .4;

            robot.outtake = gamepad1.y;
            robot.intakeNoExtend = gamepad1.right_bumper;
            robot.intakeUp = gamepad1.x;


            if (gamepad1.left_bumper) {
                robot.cycleHub = Robot.DepositTarget.mid;
            } else if (gamepad1.b) {
                robot.cycleHub = Robot.DepositTarget.shared;
            } else if (gamepad1.left_trigger > .1) {
                robot.cycleHub = Robot.DepositTarget.high;
            }

            double xyGain;
            double wGain;

            if (robot.capState != 1 || robot.depositState == 4) {
                xyGain = this.xySlowGain;
                wGain = this.wSlowGain;
            } else {
                xyGain = this.xyGain;
                wGain = this.wGain;
            }

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
