package org.firstinspires.ftc.teamcode.Teleop;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Debug.Logger;
import org.firstinspires.ftc.teamcode.RobotClasses.Deposit;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.Arrays;

import static java.lang.Math.PI;

@TeleOp(name = "1 Teleop")
@SuppressWarnings("FieldCanBeLocal")
@Config
public class Teleop extends LinearOpMode {

    // Backup Starting Position
    private final double startX = 87;
    private final double startY = 63;
    private final double startTheta = PI/2;

    private Robot robot;

    public static boolean useAutoPos = true;
    public static boolean isRed = true;

    // Control Gains
    private double xyGain = 1;
    private double wGain = 1;

    // Toggles
    private boolean slidesToggle = false, slidesDeposit = false, slidesCap = false;
    private boolean markerToggle = false, markerArmDown = false;
    private boolean servoToggle = false;
    private int depositServoStatus = 0;

    double slidesPower = 1;

    /*
    Controller Button Mappings:
    Gamepad 1
    Left Stick/Right Stick - Drivetrain Controls
    X - Reset Odo
    Left Trigger - Intake Reverse
    Right Trigger - Intake On


    Gamepad 2
    Right Trigger - Slow Mode
    A - Slides To Home Pos
    B - Slides to Mid Pos
    X - Slides To Top Pos
    Y - Slides To Capping Pos
    Right Bumper - Open Depositor Servo
    Left Bumper - Carousel Motor
    dPad Up - Team Marker Servo
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

//        robot.deposit.resetAtHomeHeight();

        robot.deposit.close();

        waitForStart();

        while (opModeIsActive()) {

            // Intake On / Rev / Off
            if (gamepad1.right_trigger > 0.1) {
                robot.intake.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.1) {
                robot.intake.setPower(-gamepad1.left_trigger);
            } else {
                robot.intake.off();
            }

            //moving slides
            if (gamepad2.a) {
                robot.deposit.close();
                depositServoStatus = 0;
                robot.deposit.moveSlides(slidesPower, Deposit.deposit_height.HOME);
            }

            if (gamepad2.b) {
                robot.deposit.hold();
                depositServoStatus = 1;
                robot.deposit.moveSlides(slidesPower, Deposit.deposit_height.MID);
            }

            if (gamepad2.x) {
                robot.deposit.hold();
                depositServoStatus = 1;
                robot.deposit.moveSlides(slidesPower, Deposit.deposit_height.TOP);
            }

            if (gamepad2.y) {
                robot.deposit.hold();
                depositServoStatus = 1;
                robot.deposit.moveSlides(slidesPower, Deposit.deposit_height.CAP);
            }

//            if (gamepad2.left_trigger > 0.1) {
//                robot.deposit.moveSlides(gamepad2.left_trigger);
//            } else {
//                robot.deposit.moveSlides(0);
//            }

            if (gamepad2.right_bumper && !servoToggle) {
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
            } else if (!gamepad2.right_bumper && servoToggle) {
                servoToggle = false;
            }

            if (gamepad2.dpad_up && !markerToggle) {
                if (markerArmDown) {
                    robot.deposit.markerArmUp();
                    markerArmDown = false;
                } else {
                    robot.deposit.markerArmDown();
                    markerArmDown = true;
                }
                markerToggle = true;
            } else if (!gamepad2.dpad_up && markerToggle) {
                markerToggle = false;
            }

            if (gamepad2.left_bumper) {
                robot.carousel.rotate();
            } else {
                robot.carousel.stop();
            }

            // Slow Mode
            if (gamepad2.right_trigger > 0) {
                xyGain = 0.22;
                wGain = 0.17;
            } else {
                xyGain = 1;
                wGain = 1;
            }

            // Reset Odometry
            if (gamepad1.x) {
                robot.resetOdo(robot.isRed ? 87 : 57, 63, PI/2);
            }

            // Drivetrain Controls
            robot.drivetrain.setControls(-gamepad1.left_stick_y * xyGain, -Math.pow(gamepad1.right_stick_x * wGain, 3));

            // Update Robot
            robot.update();

            // Telemetry
            telemetry.addData("X", robot.x);
            telemetry.addData("Y", robot.y);
            telemetry.addData("Theta", robot.theta);
            telemetry.addData("Slides Height", robot.deposit.getSlidesHeight());
            //telemetry.addData("Intake Full", robot.intake.intakeFull());
            telemetry.addData("# Cycles", robot.cycles);
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