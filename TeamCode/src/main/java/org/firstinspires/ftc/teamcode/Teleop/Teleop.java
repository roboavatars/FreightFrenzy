package org.firstinspires.ftc.teamcode.Teleop;

import static java.lang.Math.PI;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Debug.Logger;
import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Deposit;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.ArrayList;
import java.util.Arrays;

@TeleOp(name = "1 Teleop")
@SuppressWarnings("FieldCanBeLocal")
@Config
public class Teleop extends LinearOpMode {

    // Backup Starting Position
    public static double startX = 138;
    public static double startY = 122;
    public static double startTheta = PI / 2;

    public static boolean useAutoPos = false;
    public static boolean isRed = true;

    private Robot robot;

    // Control Gains
    private double xyGain;
    private double wGain;

    // Toggles
    private boolean slidesToggle = false, slidesDeposit = false, slidesCap = false;
    private boolean markerToggle = false, markerArmDown = false;
    private boolean servoToggle = false;

    private int depositServoStatus = 0;
    private double slidesPower = 1;

    // Team Marker Arm Controls
    double teamMarkerManualPos = Constants.TEAM_MARKER_UP_POS;
    double teamMarkerArmSpeed = .001;

    // Cycle counter stuff
    private ArrayList<Double> cycles = new ArrayList<>();
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

        robot.deposit.close();

        waitForStart();

        ElapsedTime cycleTimer = new ElapsedTime();
        cycleTimer.reset();

        while (opModeIsActive()) {

            // Intake On / Rev / Off
            if (gamepad2.left_trigger > 0.1) {
                robot.intake.reverse();
            } else {
                if (gamepad1.right_trigger > 0.1) {
                    robot.intake.setPower(gamepad1.right_trigger);
                } else if (gamepad1.left_trigger > 0.1) {
                    robot.intake.setPower(-gamepad1.left_trigger);
                } else {
                    robot.intake.off();
                }
            }

            // Moving Slides
            if (gamepad2.a && !cycleToggle) {
                robot.deposit.close();
                depositServoStatus = 0;
                robot.deposit.moveSlides(slidesPower, Deposit.DepositHeight.HOME);

                // cycle stuff
                cycleToggle = true;
                cycles.add(cycleTimer.seconds());
                cycleTimer.reset();
                robot.markCycle();
            } else if (!gamepad2.a && cycleToggle) {
                cycleToggle = false;
            }

            if (gamepad2.b) {
                robot.deposit.hold();
                depositServoStatus = 1;
                robot.deposit.moveSlides(slidesPower, Deposit.DepositHeight.MID);
            }

            if (gamepad2.x) {
                robot.deposit.hold();
                depositServoStatus = 1;
                robot.deposit.moveSlides(slidesPower, Deposit.DepositHeight.TOP);
            }

            // Capping Controls
            if (gamepad2.y) {
                robot.deposit.markerArmUp();
                robot.deposit.moveSlides(slidesPower, Deposit.DepositHeight.CAP);
            } else if (gamepad2.dpad_up) {
                robot.deposit.markerArmUp();
                teamMarkerManualPos = Constants.TEAM_MARKER_UP_POS;
            } else if (gamepad2.dpad_down) {
                robot.deposit.markerArmDown();
            } else if (gamepad2.right_trigger > .1) {
                teamMarkerManualPos += gamepad2.right_trigger * teamMarkerArmSpeed;
                robot.deposit.markerSetPosition(teamMarkerManualPos);
            } else if (gamepad2.left_trigger > .1) {
                teamMarkerManualPos -= gamepad2.left_trigger * teamMarkerArmSpeed;
                robot.deposit.markerSetPosition(teamMarkerManualPos);
            }

            // Deposit Servo
            if (gamepad1.right_bumper && !servoToggle) {
                if (depositServoStatus == 0) {
                    robot.deposit.hold();
                    depositServoStatus = 1;
                } else if (depositServoStatus == 1) {
                    robot.deposit.open();
                    depositServoStatus = 2;
                } else if (depositServoStatus == 2) {
                    robot.deposit.close();
                    depositServoStatus = 0;
                }
                servoToggle = true;
            } else if (!gamepad1.right_bumper && servoToggle) {
                servoToggle = false;
            }

            // Carousel
            if (gamepad2.left_bumper) {
                robot.carousel.rotateRed();
            } else if (gamepad2.dpad_down) {
                robot.carousel.rotateBlue();
            } else {
                robot.carousel.stop();
            }

            // Slow Mode
            if (gamepad2.right_trigger > 0) {
                xyGain = 0.22;
                wGain = 0.17;
            } else {
                xyGain = 1;
                wGain = 0.6;
            }

            // Drivetrain Controls
            robot.drivetrain.setControls(-gamepad1.left_stick_y * xyGain, -gamepad1.right_stick_x * wGain);

            // Update Robot
            robot.update();

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