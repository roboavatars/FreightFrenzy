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

@TeleOp(name = "0 Blue Teleop")
@SuppressWarnings("FieldCanBeLocal")
@Config
public class BlueTeleop extends LinearOpMode {

    // Backup Starting Position
    public static double startX = 138;
    public static double startY = 81;
    public static double startTheta = PI/2;

    public static boolean useAutoPos = false;
    public static boolean isRed = false;

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
            robot.turret.initialTheta = Constants.TURRET_HOME_THETA * PI;
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

            if (gamepad1.left_bumper) {
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

            if (robot.cycleHub == Robot.DepositTarget.neutral) {
//                robot.carousel.out();
            } else if (gamepad1.x && !duckToggle) {
                robot.cycleHub = Robot.DepositTarget.duck;
                duckToggle = true;
                if (!duckActive) {
                    robot.carousel.out();
//                    xyGain = 0.25;
//                    wGain = 0.3;
//                    if (isRed) robot.resetOdo(138, 18, 3 * PI/2);
//                    else robot.resetOdo(6, 18, 3 * PI/2);
                } else {
                    robot.carousel.home();
//                    xyGain = 1;
//                    wGain = 0.6;
                }
                duckActive = !duckActive;
            } else if (!gamepad1.x && duckToggle) {
                duckToggle = false;
            }


//            if (duckActive) {
//                if (Math.abs(gamepad1.right_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_y) > 0.1) {
//                    duckActive = false;
//                    robot.carousel.home();
//                } else {
//                    if (isRed) robot.setTargetPoint(130, 24, -PI / 6);
//                    else robot.setTargetPoint(14, 24, 5 * PI / 4);
//                }
//            }

            if (gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.x) {
                robot.deposit.armOffset = 0;
                robot.turret.turretOffset = 0;
                robot.deposit.slidesOffset = 0;
            }

            // Offsets
//            if (gamepad1.y) robot.intake.intakeOffset += 0.025;
//            else if (gamepad1.b) robot.intake.intakeOffset -= 0.025;

            if (robot.depositingFreight) {
                robot.turret.turretOffset += 0.008 * gamepad2.left_stick_x;
                robot.deposit.slidesOffset -= 0.16 * gamepad2.left_stick_y;
                robot.deposit.armOffset += 6 * gamepad2.right_stick_y;
            } else {
                robot.turret.initialTheta += 0.008 * gamepad2.left_stick_x;
//                robot.deposit.initialSlidesPos += Math.round(0.16 * gamepad2.left_stick_y);
//                robot.deposit.initialArmPos -= Math.round(6 * gamepad2.right_stick_y);
                if (gamepad2.right_bumper) {
                    robot.deposit.reset = true;
                } else if (gamepad2.left_bumper) {
                    robot.deposit.reset = false;
                }
            }

            if (gamepad2.a) {
                robot.resetDeposit();
            }

            // Odo Reset
            if (gamepad2.x) {
                robot.resetOdo(138,81,PI/2);
                imu.resetHeading(PI/2);
            }

            // Retract Odo
            if (gamepad2.y) {
                robot.drivetrain.retractOdo();
            }

            if (!defenseModeToggle && gamepad1.b) {
                robot.defenseMode = !robot.defenseMode;
                defenseModeToggle = true;
            } else if (defenseModeToggle && !gamepad1.b) {
                defenseModeToggle = false;
            }

            if (duckActive) {
                if (gamepad1.y) {
                    robot.carousel.on();
                    robot.deposit.hold();
                } else {
                    robot.carousel.stop();
                    robot.deposit.open();
                }
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
