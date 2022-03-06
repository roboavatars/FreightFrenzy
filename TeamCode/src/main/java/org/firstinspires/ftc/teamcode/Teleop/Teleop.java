package org.firstinspires.ftc.teamcode.Teleop;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Debug.Logger;
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

    // Control Gains
    private double xyGain;
    private double wGain;

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
    Left Trigger - Defense Mode
    Right Trigger - Slow Mode
    X - Odo Reset
    B - Retract Odo
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

        // Deposit Position Offsets
        double[] depositCoords = new double[] {
                Constants.SLIDES_DISTANCE_HIGH * Math.cos(PI * (Constants.TURRET_ALLIANCE_RED_CYCLE_HIGH_THETA + (isRed ? 0.5 : -0.5))),
                Constants.SLIDES_DISTANCE_HIGH * Math.sin(PI * (Constants.TURRET_ALLIANCE_RED_CYCLE_HIGH_THETA + (isRed ? 0.5 : -0.5)))
        };

        waitForStart();

        ElapsedTime cycleTimer = new ElapsedTime();
        cycleTimer.reset();

        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                robot.intakeApproval = true;
            } else if (gamepad1.right_bumper) {
                robot.depositApproval = true;
            } else if (gamepad1.left_trigger > 0.1) {
                robot.transferVerify = true;
            }

            if (gamepad1.right_trigger > 0) {
                robot.intakeFull = true;
            }

            // robot.intakeReverse = gamepad1.right_trigger > 0.1;

            if (gamepad1.a) {
                robot.cancelAutomation();
            }

            // Switching the Goal Cycled
            if (gamepad1.dpad_up) {
                robot.cycleHub = Robot.DepositTarget.allianceHigh;
//                depositCoords[0] = Constants.SLIDES_DISTANCE_HIGH *
//                        Math.cos(PI * (isRed ? Constants.TURRET_ALLIANCE_RED_CYCLE_HIGH_THETA + 0.5 : 0.5 - Constants.TURRET_ALLIANCE_RED_CYCLE_HIGH_THETA));
//                depositCoords[1] = Constants.SLIDES_DISTANCE_HIGH *
//                        Math.sin(PI * (isRed ? Constants.TURRET_ALLIANCE_RED_CYCLE_HIGH_THETA + 0.5 : 0.5 - Constants.TURRET_ALLIANCE_RED_CYCLE_HIGH_THETA));
            }
            else if (gamepad1.dpad_left) {
                robot.cycleHub = Robot.DepositTarget.allianceMid;
//                depositCoords[0] = Constants.SLIDES_DISTANCE_MID *
//                        Math.cos(PI * (isRed ? Constants.TURRET_ALLIANCE_RED_CYCLE_HIGH_THETA + 0.5 : 0.5 - Constants.TURRET_ALLIANCE_RED_CYCLE_HIGH_THETA));
//                depositCoords[1] = Constants.SLIDES_DISTANCE_MID *
//                        Math.sin(PI * (isRed ? Constants.TURRET_ALLIANCE_RED_CYCLE_HIGH_THETA + 0.5 : 0.5 - Constants.TURRET_ALLIANCE_RED_CYCLE_HIGH_THETA));
            }
            else if (gamepad1.dpad_down) {
                robot.cycleHub = Robot.DepositTarget.allianceLow;
//                depositCoords[0] = Constants.SLIDES_DISTANCE_LOW *
//                        Math.cos(PI * (isRed ? Constants.TURRET_ALLIANCE_RED_CYCLE_HIGH_THETA + 0.5 : 0.5 - Constants.TURRET_ALLIANCE_RED_CYCLE_HIGH_THETA));
//                depositCoords[1] = Constants.SLIDES_DISTANCE_LOW *
//                        Math.sin(PI * (isRed ? Constants.TURRET_ALLIANCE_RED_CYCLE_HIGH_THETA + 0.5 : 0.5 - Constants.TURRET_ALLIANCE_RED_CYCLE_HIGH_THETA));
            }
            else if (gamepad1.dpad_right) {
                robot.cycleHub = Robot.DepositTarget.neutral;
            }
            else if (gamepad1.x) {
                robot.cycleHub = Robot.DepositTarget.duck;
                if (!robot.carousel.isOut) robot.carousel.out();
                else robot.carousel.home();
//                depositCoords[0] = Constants.SLIDES_DISTANCE_DUCK *
//                        Math.cos(PI * (isRed ? Constants.TURRET_DUCK_RED_CYCLE_THETA + 0.5 : 0.5 - Constants.TURRET_DUCK_RED_CYCLE_THETA));
//                depositCoords[1] = Constants.SLIDES_DISTANCE_DUCK *
//                        Math.sin(PI * (isRed ? Constants.TURRET_DUCK_RED_CYCLE_THETA + 0.5 : 0.5 - Constants.TURRET_DUCK_RED_CYCLE_THETA));
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
            }

            // Retract Odo
            if (gamepad2.b) {
                robot.drivetrain.retractOdo();
            }

            if (gamepad2.left_trigger > 0.1) {
                robot.defenseMode = !robot.defenseMode;
            }

            // Field Centric Deposit Offsets
            /*if (robot.cycleHub == Robot.DepositTarget.neutral){
                robot.turret.turretOffset -= (isRed ? 0.03 : -0.03) * gamepad2.left_stick_y;
                robot.deposit.slidesOffset = 0;
            } else {
                depositCoords[0] += 0.15 * gamepad2.left_stick_x;
                depositCoords[1] += 0.15 * gamepad2.left_stick_y;

                if (robot.cycleHub == Robot.DepositTarget.allianceLow) {
                    robot.turret.turretOffset = Math.atan2(depositCoords[1], depositCoords[0])
                            - PI * (isRed ? Constants.TURRET_ALLIANCE_RED_CYCLE_LOW_THETA + 0.5 : 0.5 - Constants.TURRET_ALLIANCE_RED_CYCLE_LOW_THETA);
                    robot.deposit.slidesOffset = (int) (Deposit.DEPOSIT_SLIDES_TICKS_PER_INCH * (Math.hypot(depositCoords[0], depositCoords[1]) - Constants.SLIDES_DISTANCE_LOW));
                } else if (robot.cycleHub == Robot.DepositTarget.allianceMid) {
                    robot.turret.turretOffset = Math.atan2(depositCoords[1], depositCoords[0])
                            - PI * (isRed ? Constants.TURRET_ALLIANCE_RED_CYCLE_MID_THETA + 0.5 : 0.5 - Constants.TURRET_ALLIANCE_RED_CYCLE_MID_THETA);
                    robot.deposit.slidesOffset = (int) (Deposit.DEPOSIT_SLIDES_TICKS_PER_INCH * (Math.hypot(depositCoords[0], depositCoords[1]) - Constants.SLIDES_DISTANCE_MID));
                } else if (robot.cycleHub == Robot.DepositTarget.allianceHigh) {
                    robot.turret.turretOffset = Math.atan2(depositCoords[1], depositCoords[0])
                            - PI * (isRed ? Constants.TURRET_ALLIANCE_RED_CYCLE_HIGH_THETA + 0.5 : 0.5 - Constants.TURRET_ALLIANCE_RED_CYCLE_HIGH_THETA);
                    robot.deposit.slidesOffset = (int) (Deposit.DEPOSIT_SLIDES_TICKS_PER_INCH * (Math.hypot(depositCoords[0], depositCoords[1]) - Constants.SLIDES_DISTANCE_HIGH));
                } else if (robot.cycleHub == Robot.DepositTarget.duck) {
                    robot.turret.turretOffset = Math.atan2(depositCoords[1], depositCoords[0])
                            - PI * (isRed ? Constants.TURRET_DUCK_RED_CYCLE_THETA + 0.5 : 0.5 - Constants.TURRET_DUCK_RED_CYCLE_THETA);
                    robot.deposit.slidesOffset = (int) (Deposit.DEPOSIT_SLIDES_TICKS_PER_INCH * (Math.hypot(depositCoords[0], depositCoords[1]) - Constants.SLIDES_DISTANCE_DUCK));
                }
            }
            robot.turret.turretOffset = -1 * robot.turret.turretOffset;*/

            // Slow Mode
            if (gamepad2.right_trigger > 0.1) {
                xyGain = 0.22;
                wGain = 0.17;
            } else {
                xyGain = 1;
                wGain = 0.6;
            }

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
            robot.drivetrain.setControls(-gamepad1.left_stick_y * xyGain, -gamepad1.left_stick_x * xyGain, -gamepad1.right_stick_x * wGain);

            // Update Robot
            robot.update();

            // Telemetry
//            for (int i = 0; i < robot.cycles.size(); i++) {
//                telemetry.addData("cycle " + i, cycles.get(i));
//            }

            telemetry.addData("X", robot.x);
            telemetry.addData("Y", robot.y);
            telemetry.addData("Theta", robot.theta);
            telemetry.addData("# Cycles", robot.cycles.size());
            telemetry.addData("Average Cycle Time", robot.cycleAvg + "s");
            telemetry.update();
        }
//        if (robot.cycleList.size() > 0) {
//            Log.w("cycle-log", "Avg dropping longest: " + ((robot.cycleTotal - robot.longestCycle) / (robot.cycles - 1)) + "s");
//        }
        robot.stop();
    }
}
