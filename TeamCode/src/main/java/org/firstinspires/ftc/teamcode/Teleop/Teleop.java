package org.firstinspires.ftc.teamcode.Teleop;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Debug.Logger;
import org.firstinspires.ftc.teamcode.Localization.IMU;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.Arrays;

@TeleOp(name = "1 Teleop")
@Config
public class Teleop extends LinearOpMode {

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
    private double wGain = 1;

    // Rumbles
    private boolean teleRumble1 = false;
    private boolean midTeleRumble = false;
    private boolean endgameRumble = false;

    private double starCarouselTime = 0;

    private double capUpOffset = 0;
    private double capDownOffset = 0;
    private boolean cappingDown = true;

    /*
    Controller Button Mappings: *updated 3/13/22 12:30 PM*
    gamepad 1:
    y - carousel on
    left bumper - cancel automation
    b - defense mode
    x - duck cycle
    dpad up - high goal
    dpad left - mid goal
    dpad down - low goal
    dpad right - neutral

    gamepad 2:
    left trigger - deposit
    right trigger - intake
    right bumper - enable deposit reset mode
    left bumper - disable deposit reset mode
    left stick x - turret offset
    left stick y - slides offset
    right stick y - arm offset
    a = reset deposit offsets
    x - reset odo/imu
    y - retract odo
    */

    @Override
    public void runOpMode() throws InterruptedException {
        if (useAutoPos) {
            double[] initialData = Logger.readPos();
            telemetry.addData("Starting Data", Arrays.toString(initialData));
            telemetry.update();
            Robot.log("Starting Data " + Arrays.toString(initialData));
            robot = new Robot(this, initialData[1], initialData[2], initialData[3], false, initialData[0] == 1, true, initialData[4], (int) initialData[5], (int) initialData[6]);
            robot.logger.startLogging(false, initialData[0] == 1);
        } else {
            robot = new Robot(this, (isRed ? startX : 144 - startX), startY, startTheta, false, isRed);
            robot.logger.startLogging(false, isRed);
        }

//        imu = new IMU(robot.theta, this);
        waitForStart();

        ElapsedTime cycleTimer = new ElapsedTime();
        cycleTimer.reset();

        while (opModeIsActive()) {
            robot.intakeApproval = gamepad1.right_trigger > .1;
            robot.depositApproval = gamepad1.left_trigger > .1;


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
