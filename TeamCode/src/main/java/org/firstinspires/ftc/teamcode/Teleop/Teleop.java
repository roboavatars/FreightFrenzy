package org.firstinspires.ftc.teamcode.Teleop;

import static java.lang.Math.PI;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Debug.Logger;
import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.ArrayList;
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

    // Toggles
    private boolean servoToggle = false;

    // Cycle counter stuff
    private final ArrayList<Double> cycles = new ArrayList<>();

    // Rumbles
    private boolean teleRumble1 = false;
    private boolean midTeleRumble = false;
    private boolean endgameRumble = false;

    /*
    Controller Button Mappings:
    Gamepad 1
    Left Stick/Right Stick - Drivetrain Controls
    Left Bumper - Auto intake
    Right Bumper - Approve freight deposit
    A - Cancel automation

    Gamepad 2
    Left Bumper - Carousel Motor Red
    Right Trigger - Slow Mode
    A - Increase slide extend
    Y - Decrease slide extend
    Dpad Up - Increase arm position
    Dpad Down - Decrease arm position
    Dpad Left - Decrease turret theta
    Dpad Right - Increase turret theta
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

        waitForStart();

        ElapsedTime cycleTimer = new ElapsedTime();
        cycleTimer.reset();

        while (opModeIsActive()) {

            if (gamepad1.left_bumper) {
                robot.intakeApproval = true;
            } else if (gamepad1.right_bumper) {
                robot.depositApproval = true;
            }
            if (gamepad1.left_trigger>0){
                robot.intakeFull = true;
            }

            if (gamepad1.a) {
                robot.cancelAutomation();
            }

            if (gamepad1.dpad_up) robot.cycleHub = Robot.DepositTarget.allianceHigh;
            else if (gamepad1.dpad_left) robot.cycleHub = Robot.DepositTarget.allianceMid;
            else if (gamepad1.dpad_down) robot.cycleHub = Robot.DepositTarget.allianceLow;
            else if (gamepad1.dpad_right) robot.cycleHub = Robot.DepositTarget.neutral;
            else if (gamepad1.y) robot.cycleHub = Robot.DepositTarget.duck;

            if (gamepad2.dpad_up) Constants.DEPOSIT_ARM_HIGH -= 2;
            else if (gamepad2.dpad_down) Constants.DEPOSIT_ARM_HIGH += 2;

            if (gamepad2.dpad_right) {
                if (robot.cycleHub == Robot.DepositTarget.allianceHigh || robot.cycleHub == Robot.DepositTarget.allianceMid
                        || robot.cycleHub == Robot.DepositTarget.allianceLow) {
                    Constants.TURRET_ALLIANCE_RED_CYCLE_THETA -= 0.005;
                } else if (robot.cycleHub == Robot.DepositTarget.neutral) {
                    Constants.TURRET_NEUTRAL_RED_CYCLE_THETA -= 0.005;
                } else if (robot.cycleHub == Robot.DepositTarget.duck) {
                    Constants.TURRET_DUCK_RED_CYCLE_THETA -= 0.005;
                }
            } else if (gamepad2.dpad_left) {
                if (robot.cycleHub == Robot.DepositTarget.allianceHigh || robot.cycleHub == Robot.DepositTarget.allianceMid
                        || robot.cycleHub == Robot.DepositTarget.allianceLow) {
                    Constants.TURRET_ALLIANCE_RED_CYCLE_THETA += 0.005;
                } else if (robot.cycleHub == Robot.DepositTarget.neutral) {
                    Constants.TURRET_NEUTRAL_RED_CYCLE_THETA += 0.005;
                } else if (robot.cycleHub == Robot.DepositTarget.duck) {
                    Constants.TURRET_DUCK_RED_CYCLE_THETA += 0.005;
                }
            }

            if (gamepad2.y) {
                if (robot.cycleHub == Robot.DepositTarget.allianceHigh) Constants.SLIDES_DISTANCE_HIGH += 0.2;
                else if (robot.cycleHub == Robot.DepositTarget.allianceMid) Constants.SLIDES_DISTANCE_MID += 0.2;
                else if (robot.cycleHub == Robot.DepositTarget.allianceLow) Constants.SLIDES_DISTANCE_LOW += 0.2;
                else if (robot.cycleHub == Robot.DepositTarget.duck) Constants.SLIDES_DISTANCE_DUCK += 0.2;
            } else if (gamepad2.a) {
                if (robot.cycleHub == Robot.DepositTarget.allianceHigh) Constants.SLIDES_DISTANCE_HIGH -= 0.2;
                else if (robot.cycleHub == Robot.DepositTarget.allianceMid) Constants.SLIDES_DISTANCE_MID -= 0.2;
                else if (robot.cycleHub == Robot.DepositTarget.allianceLow) Constants.SLIDES_DISTANCE_LOW -= 0.2;
                else if (robot.cycleHub == Robot.DepositTarget.duck) Constants.SLIDES_DISTANCE_DUCK -= 0.2;
            }

            // Slow Mode
            if (gamepad2.right_trigger > 0) {
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
            for (int i = 0; i < cycles.size(); i++) {
                telemetry.addData("cycle " + i, cycles.get(i));
            }

            telemetry.addData("X", robot.x);
            telemetry.addData("Y", robot.y);
            telemetry.addData("Theta", robot.theta);
            telemetry.addData("# Cycles", cycles.size());
            telemetry.addData("Average Cycle Time", (robot.cycleTotal / robot.cycles) + "s");
            telemetry.update();
        }

        Log.w("cycle-log", "# Cycles: " + robot.cycles);
        Log.w("cycle-log", "Avg cycle Time: " + (robot.cycleTotal / robot.cycles) + "s");
        if (robot.cycles > 0) {
            Log.w("cycle-log", "Avg dropping longest: " + ((robot.cycleTotal - robot.longestCycle) / (robot.cycles - 1)) + "s");
        }
        robot.stop();
    }
}