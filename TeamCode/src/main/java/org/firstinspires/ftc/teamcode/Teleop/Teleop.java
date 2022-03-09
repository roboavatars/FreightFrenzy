package org.firstinspires.ftc.teamcode.Teleop;

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

@TeleOp(name = "0 Teleop")
@SuppressWarnings("FieldCanBeLocal")
@Config
public class Teleop extends LinearOpMode {

    // Backup Starting Position
    public static double startX = 138;
    public static double startY = 81;
    public static double startTheta = PI/2;

    public static boolean useAutoPos = false;
    public static boolean isRed = true;

    private Robot robot;
    private IMU imu;

    // Toggles
    private boolean duckActive = false;
    private boolean duckToggle = false;
    private boolean intakeToggle = false;
    private boolean defenseModeToggle = false;

    // Control Gains
    private double xyGain = 1;
    private double wGain = 0.6;

    // Rumbles
    private boolean teleRumble1 = false;
    private boolean midTeleRumble = false;
    private boolean endgameRumble = false;

    /*
    Controller Button Mappings:
    Gamepad 1
    Left/Right Sticks - Drivetrain Controls
    Left Bumper - Start Auto Intake
    Right Bumper - Approve Freight Deposit
    Left Trigger - Verify Transfer
    Right Trigger - Override Intake Sensor
    Dpad Up - High Goal
    Dpad Left - Mid Goal
    Dpad Down - Low Goal
    Dpad Right - Neutral Goal
    X - Duck Cycling
    A - Cancel Automation
    Y/B - Intake Slides Offset

    Gamepad 2
    Left Stick - Turret, Deposit Slides Offset
    Right Stick - Arm Offset
    Right Trigger - Slow Mode
    X - Odo Reset
    Y - Retract Odo
    B - Defense Mode
    A - Carousel On/Off
    */

    @Override
    public void runOpMode() {
        if (useAutoPos) {
            double[] initialData = Logger.readPos();
            telemetry.addData("Starting Position", Arrays.toString(initialData));
            telemetry.update();
            robot = new Robot(this, initialData[1], initialData[2], initialData[3], false, initialData[0] == 1);
            robot.logger.startLogging(false, initialData[0] == 1);
        } else {
            robot = new Robot(this, (isRed ? startX : 144 - startX), startY, startTheta, false, isRed);
            robot.logger.startLogging(false, isRed);
        }
        imu = new IMU(robot.theta, this);

        // Deposit Position Offsets
        double[] depositCoords = new double[] {
                Constants.SLIDES_DISTANCE_HIGH * Math.cos(PI * (Constants.TURRET_ALLIANCE_RED_CYCLE_HIGH_THETA + (isRed ? 0.5 : -0.5))),
                Constants.SLIDES_DISTANCE_HIGH * Math.sin(PI * (Constants.TURRET_ALLIANCE_RED_CYCLE_HIGH_THETA + (isRed ? 0.5 : -0.5)))
        };

        waitForStart();

        ElapsedTime cycleTimer = new ElapsedTime();
        cycleTimer.reset();

        while (opModeIsActive()) {
            if (gamepad2.left_trigger > 0.1) {
                robot.depositApproval = true;
            }

            if (!intakeToggle && gamepad2.right_trigger > 0.1) {
                robot.intakeApproval = !robot.intakeApproval;
                intakeToggle = true;
            } else if (intakeToggle && gamepad2.right_trigger <= 0.1) {
                intakeToggle = false;
            }

            // robot.intakeReverse = gamepad1.right_trigger > 0.1;

            if (gamepad1.a) {
                robot.cancelAutomation();
            }

            // Switching the Goal Cycled
            if (gamepad1.dpad_up) {
                robot.cycleHub = Robot.DepositTarget.allianceHigh;
            } else if (gamepad1.dpad_left) {
                robot.cycleHub = Robot.DepositTarget.allianceMid;
            } else if (gamepad1.dpad_down) {
                robot.cycleHub = Robot.DepositTarget.allianceLow;
            } else if (gamepad1.dpad_right) {
                robot.cycleHub = Robot.DepositTarget.neutral;
            }

            if ( robot.cycleHub == Robot.DepositTarget.neutral) {
                robot.carousel.out();
            } else if (gamepad1.x && !duckToggle) {
                robot.cycleHub = Robot.DepositTarget.duck;
                duckToggle = true;
                if (!duckActive) {
                    robot.carousel.out();
                    xyGain = 0.25;
                    wGain = 0.3;
                } else {
                    robot.carousel.home();
                    xyGain = 1;
                    wGain = 0.6;
                }
                duckActive = !duckActive;
            } else if (!gamepad1.x && duckToggle) {
                duckToggle = false;
            }

            if (gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.x) {
                robot.deposit.armOffset = 0;
                robot.turret.turretOffset = 0;
                robot.deposit.slidesOffset = 0;
            }

            // Offsets
            if (gamepad1.y) robot.intake.intakeOffset += 0.025;
            else if (gamepad1.b) robot.intake.intakeOffset -= 0.025;

            robot.turret.turretOffset += 0.008 * gamepad2.left_stick_x;
            robot.deposit.slidesOffset -= 1.5 * gamepad2.left_stick_y;
            robot.deposit.armOffset += 2 * gamepad2.right_stick_y;

            // Odo Reset
            if (gamepad2.x) {
                robot.resetOdo(138,81,PI/2);
                imu.resetHeading(PI/2);
            }

            // Retract Odo
            if (gamepad2.y) {
                robot.drivetrain.retractOdo();
            }

            if (!defenseModeToggle && gamepad2.b) {
                robot.defenseMode = !robot.defenseMode;
                defenseModeToggle = true;
            } else if (defenseModeToggle && !gamepad2.b) {
                defenseModeToggle = false;
            }

            if (gamepad2.a) {
                robot.carousel.on();
            } else {
                robot.carousel.stop();
            }

//            // Slow Mode
//            if (gamepad2.right_trigger > 0.1) {
//                xyGain = 0.22;
//                wGain = 0.17;
//            } else if (!duckActive) {
//                xyGain = 1;
//                wGain = 0.6;
//            }

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

            // Drivetrain Controls
            // Field Centric Driving
            imu.updateHeading();
            double theta = imu.getTheta() + (isRed? 0 : PI);
            double xControls = gamepad1.left_stick_x * xyGain;
            double yControls = gamepad1.left_stick_y * xyGain;
            robot.drivetrain.setControls(xControls * Math.sin(theta) + yControls * Math.cos(theta), xControls * Math.cos(theta) - yControls * Math.sin(theta), -gamepad1.right_stick_x * wGain);

            // Update Robot
            robot.update();

            // Telemetry
//            for (int i = 0; i < robot.cycles.size(); i++) {
//                telemetry.addData("cycle " + i, cycles.get(i));
//            }

            telemetry.addData("X", Robot.round(robot.x));
            telemetry.addData("Y", Robot.round(robot.y));
            telemetry.addData("Theta", Robot.round(robot.theta));
            telemetry.addData("Cycle Hub", robot.cycleHub);
            telemetry.addData("Defense Mode", robot.defenseMode);
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
