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
    private int depositServoStatus = 0;

    // Cycle counter stuff
    private final ArrayList<Double> cycles = new ArrayList<>();
    private boolean cycleToggle = false;

    // Rumbles
    private boolean teleRumble1 = false;
    private boolean midTeleRumble = false;
    private boolean endgameRumble = false;

    /*
    Controller Button Mappings:
    Gamepad 1
    Left Stick/Right Stick - Drivetrain Controls
    Left Trigger - Intake Reverse
    Right Trigger - Intake On


    Gamepad 2
    Right Trigger - Slow Mode
    Left Trigger - Intake Override
    A - Slides To Home Pos
    B - Slides to Mid Pos
    X - Slides To Top Pos
    Y - Slides To Capping Pos
    Right Bumper - Open Depositor Servo
    Left Bumper - Carousel Motor Red
    Dpad Down - Carousel Motor Blue
    Dpad Up - Team Marker Servo
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
            }

            if (gamepad1.right_bumper) {
                robot.depositApproval = true;
            }

            if (gamepad2.dpad_up) Constants.DEPOSIT_ARM_HIGH -= 2;
            else if (gamepad2.dpad_down) Constants.DEPOSIT_ARM_HIGH += 2;

            if (gamepad2.dpad_right){
                if (robot.cycleHub == Robot.hub.allianceHigh
                        || robot.cycleHub == Robot.hub.allianceMid
                        || robot.cycleHub == Robot.hub.allianceLow){
                    Constants.TURRET_ALLIANCE_RED_CYCLE_THETA -= 0.005;
                } else if (robot.cycleHub == Robot.hub.neutral){
                    Constants.TURRET_NEUTRAL_RED_CYCLE_THETA -= 0.005;
                } else if (robot.cycleHub == Robot.hub.duck){
                    Constants.TURRET_DUCK_RED_CYCLE_THETA -= 0.005;
                }
            } else if (gamepad2.dpad_left){
                if (robot.cycleHub == Robot.hub.allianceHigh
                        || robot.cycleHub == Robot.hub.allianceMid
                        || robot.cycleHub == Robot.hub.allianceLow){
                    Constants.TURRET_ALLIANCE_RED_CYCLE_THETA += 0.005;
                } else if (robot.cycleHub == Robot.hub.neutral){
                    Constants.TURRET_NEUTRAL_RED_CYCLE_THETA += 0.005;
                } else if (robot.cycleHub == Robot.hub.duck){
                    Constants.TURRET_DUCK_RED_CYCLE_THETA += 0.005;
                }
            }

            if (gamepad2.y){
                if (robot.cycleHub == Robot.hub.allianceHigh) Constants.SLIDES_DISTANCE_HIGH += 0.2;
                else if (robot.cycleHub == Robot.hub.allianceMid) Constants.SLIDES_DISTANCE_MID += 0.2;
                else if (robot.cycleHub == Robot.hub.allianceLow) Constants.SLIDES_DISTANCE_LOW += 0.2;
                else if (robot.cycleHub == Robot.hub.duck) Constants.SLIDES_DISTANCE_DUCK += 0.2;
            }
            else if (gamepad2.a){
                if (robot.cycleHub == Robot.hub.allianceHigh) Constants.SLIDES_DISTANCE_HIGH -= 0.2;
                else if (robot.cycleHub == Robot.hub.allianceMid) Constants.SLIDES_DISTANCE_MID -= 0.2;
                else if (robot.cycleHub == Robot.hub.allianceLow) Constants.SLIDES_DISTANCE_LOW -= 0.2;
                else if (robot.cycleHub == Robot.hub.duck) Constants.SLIDES_DISTANCE_DUCK -= 0.2;
            }

            // Intake On / Off / Transfer
//            if (gamepad1.right_bumper) {
//                robot.intake.extend();
//                robot.intake.on();
//                robot.intake.flipDown();
//            } else if (gamepad1.left_bumper) {
//                robot.intake.home();
//                robot.intake.flipUp();
//                robot.intake.off();
//            } else if (gamepad1.a) {
//                robot.deposit.open();
//                robot.intake.reverse();
//            } else {
//                robot.intake.off();
//            }

            // Deposit Controls
//            if (gamepad1.x) {
//                robot.deposit.hold();
//                robot.deposit.setArmControls(Constants.DEPOSIT_ARM_HIGH);
//                robot.deposit.setSlidesControls((int) (24.9 * Deposit.DEPOSIT_SLIDES_TICKS_PER_INCH));
//                turretHome = false;
//            } else if (gamepad1.y) {
//                robot.deposit.open();
//            } else if (gamepad1.b) {
//                robot.deposit.setArmControls(Constants.DEPOSIT_ARM_OVER_SLIDES_MOTOR);
//                robot.deposit.setSlidesControls(0);
//                turretHome = true;
//            }

//            if (turretHome){
//                robot.turret.setTurretTheta(PI/2);
//            } else {
//                robot.turret.setTurretTheta(PI/4);
//            }

            /*

            // Move Back Home
            if (gamepad2.a && !cycleToggle) {
                robot.deposit.open();
                depositServoStatus = 0;
                robot.depositHome();

                // cycle stuff
                cycleToggle = true;
                cycles.add(cycleTimer.seconds());
                cycleTimer.reset();
                robot.markCycle();
            } else if (!gamepad2.a && cycleToggle) {
                cycleToggle = false;
            }

            // Set Deposit to Track Alliance Hub
            if (gamepad2.x) {
                robot.deposit.close();
                depositServoStatus = 1;
                robot.depositAllianceHub(Deposit.DepositHeight.HIGH);
            }

            // Set Deposit to Track Shared Hub
            if (gamepad2.y) {
                robot.deposit.close();
                depositServoStatus = 1;
                robot.depositTrackSharedHub();
            }

            // Carousel
            if (gamepad2.left_bumper) {
                robot.carousel.rotateRed();
            } else if (gamepad2.dpad_down) {
                robot.carousel.rotateBlue();
            } else {
                robot.carousel.stop();
            }
             */

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
            telemetry.addData("Slides Height", robot.deposit.targetHeight);
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