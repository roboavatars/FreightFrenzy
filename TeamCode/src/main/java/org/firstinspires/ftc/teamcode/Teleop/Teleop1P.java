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
    private double xyGain = 1;
    private double wGain = 0.8;

    private double xySlowGain = 0.3;
    private double wSlowGain = 0.4;

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
            robot.depositApproval = gamepad1.a || gamepad1.b || gamepad1.y;

            if (robot.intakeApproval && gamepad1.right_trigger <= .1) robot.intakeApproval = false;
            else if (gamepad1.right_trigger > .1 && !intakeApprovalToggle) {
                intakeApprovalToggle = true;
                robot.intakeApproval = true;
            } else if (gamepad1.right_trigger <= .1 && intakeApprovalToggle) {
                intakeApprovalToggle = false;
            }
            robot.outtake = gamepad1.left_bumper;

            if (gamepad1.dpad_up) robot.deposit.initialSlidesPos -= .4;
            if (gamepad1.dpad_down) robot.deposit.initialSlidesPos += .4;

            robot.outtake = gamepad1.left_bumper;
            robot.intakeNoExtend = gamepad1.right_bumper;
            robot.intakeUp = gamepad1.x;


            if (gamepad1.b) {
                robot.cycleHub = Robot.DepositTarget.mid;
            } else if (gamepad1.a) {
                robot.cycleHub = Robot.DepositTarget.low;
            } else if (gamepad1.y) {
                robot.cycleHub = Robot.DepositTarget.high;
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
