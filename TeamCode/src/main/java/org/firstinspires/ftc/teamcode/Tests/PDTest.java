package org.firstinspires.ftc.teamcode.Tests;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@TeleOp
@Config
public class PDTest extends LinearOpMode {
    public static double xKp = 0.45;
    public static double yKp = 0.34;
    public static double thetaKp = 2.5;
    public static double xKd = 0.035;
    public static double yKd = 0.0315;
    public static double thetaKd = 0.12;

    public static double targetX = 0;
    public static double targetY = 0;
    public static double targetTheta = 0;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, 78, 63, PI/2, false, true);

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.x) {
                robot.drivetrain.resetOdo(78, 63, PI/2);
            }

            if (gamepad1.dpad_left) {
                robot.setTargetPoint(30, 63, PI/2, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else if (gamepad1.dpad_right) {
                robot.setTargetPoint(126, 63, PI/2, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else if (gamepad1.dpad_up) {
                robot.setTargetPoint(78, 111, PI/2, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else if (gamepad1.dpad_down) {
                robot.setTargetPoint(78, 15, PI/2, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else if (gamepad1.left_bumper) {
                robot.setTargetPoint(78, 63, PI, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else if (gamepad1.right_bumper) {
                robot.setTargetPoint(78, 63, 0, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else if (gamepad1.a) {
                robot.setTargetPoint(targetX, targetY, targetTheta, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else if (gamepad1.b) {
                robot.setTargetPoint(111, 63, 1.91, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else {
                robot.drivetrain.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            }

            robot.update();
        }
    }
}
