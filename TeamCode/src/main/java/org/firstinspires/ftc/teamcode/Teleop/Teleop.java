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

import java.util.Arrays;

@TeleOp(name = "1 Teleop")
@SuppressWarnings("FieldCanBeLocal")
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
    public void runOpMode() {
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

        imu = new IMU(robot.theta, this);
        waitForStart();

        ElapsedTime cycleTimer = new ElapsedTime();
        cycleTimer.reset();

        while (opModeIsActive()) {
            robot.depositApproval = gamepad1.left_trigger > 0.1;
            robot.intakeApproval = gamepad1.right_trigger > 0.1;

//            if (!intakeToggle && gamepad1.right_trigger > 0.1) {
//                robot.intakeApproval = !robot.intakeApproval;
//                intakeToggle = true;
//            } else if (intakeToggle && gamepad1.right_trigger <= 0.1) {
//                intakeToggle = false;
//            }

            if (gamepad1.b) {
                robot.setCycleHub(Robot.DepositTarget.neutral);
            }

            if (robot.cycleHub != Robot.DepositTarget.high && gamepad1.left_trigger > 0.1) {
                robot.setCycleHub(Robot.DepositTarget.high);
                capDownOffset = 0;
                capUpOffset = 0;
            }

//            //Capping
//            if (gamepad1.dpad_down) {
//                robot.setCycleHub(Robot.DepositTarget.capping);
//            }
//
//            if (gamepad1.dpad_up && robot.cycleHub == Robot.DepositTarget.capping) {
//                robot.depositState = 2;
//            }
//
//            if (robot.cycleHub == Robot.DepositTarget.capping) {
//                if (gamepad1.right_bumper) {
//                    if (robot.depositState == 1) robot.capDownOffset -= 0.75;
//                    else robot.capUpOffset--;
//                } else if (gamepad1.left_bumper) {
//                    if (robot.depositState == 1) robot.capDownOffset += 0.75;
//                    else robot.capUpOffset++;
//                }
//            }

            if (gamepad1.dpad_down) {
                cappingDown = true;
                robot.setCycleHub(Robot.DepositTarget.capping);
            } else if (gamepad1.dpad_up) {
                cappingDown = false;
                robot.setCycleHub(Robot.DepositTarget.capping);
            }

            if (robot.cycleHub == Robot.DepositTarget.capping) {
                if (gamepad1.right_bumper) {
                    if (cappingDown) capDownOffset += 0.001;
                    else capUpOffset += 0.001;
                } else if (gamepad1.left_bumper) {
                    if (cappingDown) capDownOffset -= 0.001;
                    else capUpOffset -= 0.001;
                }
                if (cappingDown) {
                    robot.arm.cap(Math.max(Math.min(Constants.SERVO_CAP_DOWN + capDownOffset, 1), 0));
                } else {
                    robot.arm.cap(Math.max(Math.min(Constants.SERVO_CAP_UP + capUpOffset, 1), 0));
                }
            } else {
                robot.arm.cap(Constants.SERVO_CAP_HOME);
            }

            // Odo Reset
            if (gamepad1.x) {
                robot.resetOdo(138, 81, PI / 2);
                imu.resetHeading(PI / 2);
            }

            if (gamepad1.y) {
                if (robot.curTime - starCarouselTime < Constants.CAROUSEL_SPEED_UP_THRESHOLD)
                    robot.carousel.on(Constants.CAROUSEL_VELOCITY_SLOW);
                else
                    robot.carousel.on(Constants.CAROUSEL_VELOCITY_FAST);
            } else {
                robot.carousel.stop();
                starCarouselTime = robot.curTime;
            }

            addPacket("carousel start time", starCarouselTime);
            addPacket("robot time", robot.curTime);


            // Rumble
            if (!teleRumble1 && 120 - (System.currentTimeMillis() - robot.startTime) / 1000 < 90) {
                gamepad1.rumble(0.5, 0.5, 1000);
                teleRumble1 = true;
            }
            if (!midTeleRumble && 120 - (System.currentTimeMillis() - robot.startTime) / 1000 < 60) {
                gamepad1.rumble(0.5, 0.5, 1000);
                midTeleRumble = true;
            }
            if (!endgameRumble && 120 - (System.currentTimeMillis() - robot.startTime) / 1000 < 35) {
                gamepad1.rumble(0.5, 0.5, 1000);
                endgameRumble = true;
            }

            if (robot.cycleHub == Robot.DepositTarget.capping) {
                xyGain = 0.6;
                wGain = 0.5;
            } else {
                xyGain = 1;
                wGain = 0.7;
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
