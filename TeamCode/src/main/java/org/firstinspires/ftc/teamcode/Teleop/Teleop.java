package org.firstinspires.ftc.teamcode.Teleop;

import static java.lang.Math.PI;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Debug.Logger;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.Arrays;

@TeleOp(name = "1 Teleop")
@SuppressWarnings("FieldCanBeLocal")
@Config
public class Teleop extends LinearOpMode {

    // Backup Starting Position
    private final int startX = 87;
    private final int startY = 63;
    private final double startTheta = PI/2;

    private Robot robot;

    public static boolean robotCentric = true;
    public static boolean useAutoPos = true;
    public static boolean isRed = true;

    // Control Gains
    private double xyGain = 1;
    private double wGain = 1;

    // Toggles
    private boolean slidesToggle = false, slidesDeposit = false, slidesCap = false;
    private boolean depositorToggle = false, depositorOpen = false;

    /*
    Controller Button Mappings:
    Gamepad 1
    Left Stick/Right Stick - Drivetrain Controls
    X - Reset Odo
    Left Trigger - Intake Reverse
    Right Trigger - Intake On
    A - Slides To Home Pos
    B - Slides To Deposit Pos
    Y - Slides To Capping Pos

    Gamepad 2
    Dpad Left - Decrease Theta Offset
    Dpad Right - Increase Theta Offset
    Dpad Up - Increase Velocity Scale
    Dpad Down - Decrease Velocity Scale
    Left Trigger - Slow Mode
     */

    @Override
    public void runOpMode() {
        if (useAutoPos) {
            double[] initialData = Logger.readPos();
            telemetry.addData("Starting Position", Arrays.toString(initialData));
            telemetry.update();
            robot = new Robot(this, initialData[1], initialData[2], initialData[3], initialData[4], false, initialData[0] == 1);
            robot.logger.startLogging(false, initialData[0] == 1);
        } else {
            robot = new Robot(this, (isRed ? startX : 144 - startX), startY, startTheta, false, isRed);
            robot.logger.startLogging(false, isRed);
        }

        robot.slides.resetAtHomeHeight();

        waitForStart();

        robot.drivetrain.updateThetaError();

        while (opModeIsActive()) {

            // Intake On / Rev / Off
            if (gamepad1.right_trigger > 0) {
                robot.intake.on();
            } else if (gamepad1.left_trigger > 0) {
                robot.intake.reverse();
            } else {
                robot.intake.off();
            }

            //moving slides
            if (gamepad1.a && !slidesToggle) {  //move slides to home
                if (slidesDeposit || slidesCap) {
                    slidesToggle = true;

                    robot.slides.home();

                    slidesDeposit = false;
                    slidesCap = false;
                }
            } else if (gamepad1.b && !slidesToggle) {   //move slides to deposit pos
                if (slidesCap || (!slidesDeposit && !slidesCap)) {
                    slidesToggle = true;

                    robot.slides.deposit();

                    slidesDeposit = true;
                    slidesCap = false;
                }
            } else if (gamepad1.y && !slidesToggle) {   //move slides to capping pos
                if (slidesDeposit || (!slidesDeposit && !slidesCap)) {
                    slidesToggle = true;

                    robot.slides.cap();

                    slidesDeposit = false;
                    slidesCap = true;
                }
            } else if (!(gamepad1.a || gamepad1.b || gamepad1.y) && slidesToggle) {
                slidesToggle = false;
            }


            //toggle intake open/close
            if (robot.intake.freightIntaked()) {
                robot.intake.close();
            } else {
                robot.intake.open();
            }

            //run carousel servo
            if (gamepad2.a){
                robot.carousel.rotate();
            } else {
                robot.carousel.stop();
            }

            //toggle depositor open/close
            if (gamepad2.a && !depositorToggle) {
                if (depositorOpen){
                    robot.depositor.close();
                    depositorOpen = false;
                } else {
                    robot.depositor.open();
                    depositorOpen = true;
                }
            } else if (!gamepad2.a && depositorToggle) {
                depositorToggle = false;
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
                robot.thetaOffset = 0.03;
                robot.velocityFactor = 0.96;
            }

            // Change Shooting Theta Offset to Compensate for Odometry Drift
            if (gamepad2.dpad_left) {
                robot.thetaOffset -= 0.006;
            } else if (gamepad2.dpad_right) {
                robot.thetaOffset += 0.006;
            }

            // Change Velocity Scaling
            if (gamepad2.dpad_up) {
                robot.velocityFactor += 0.001;
            } else if (gamepad2.dpad_down) {
                robot.velocityFactor -= 0.001;
            }

            // Drivetrain Controls
            if (robotCentric) {
                robot.drivetrain.setControls(-gamepad1.left_stick_y * xyGain, -gamepad1.left_stick_x * xyGain, -gamepad1.right_stick_x * wGain);
            } else {
                robot.drivetrain.setGlobalControls((isRed ? 1 : -1) * gamepad1.left_stick_y * xyGain, (isRed ? 1 : -1) * gamepad1.left_stick_x * xyGain, -gamepad1.right_stick_x * wGain);
            }

            // Update Robot
            robot.update();

            // Telemetry
            telemetry.addData("X", robot.x);
            telemetry.addData("Y", robot.y);
            telemetry.addData("Theta", robot.theta);
            telemetry.addData("freight intaked: ", robot.intake.freightIntaked());
            telemetry.addData("Theta Offset", robot.thetaOffset);
//            telemetry.addData("Shooter Velocity", robot.shooter.getFlywheelVelocity());
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