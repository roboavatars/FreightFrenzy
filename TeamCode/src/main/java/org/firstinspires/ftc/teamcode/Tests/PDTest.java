package org.firstinspires.ftc.teamcode.Tests;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Drivetrain;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@TeleOp
@Config
public class PDTest extends LinearOpMode {
    public static double xKp = Drivetrain.xKp;
    public static double yKp = Drivetrain.yKp;
    public static double thetaKp = Drivetrain.thetaKp;
    public static double xKd = Drivetrain.xKd;
    public static double yKd = Drivetrain.yKd;
    public static double thetaKd = Drivetrain.thetaKd;

    public static double targetX = 0;
    public static double targetY = 0;
    public static double targetTheta = 0;
    public static double displacement = 48;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, 0, 0, PI / 2, true, true);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x) {
                robot.drivetrain.resetOdo(0, 0, PI / 2);
            }

            if (gamepad1.dpad_left) {
                robot.setTargetPoint(-displacement, 0, PI / 2, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else if (gamepad1.dpad_right) {
                robot.setTargetPoint(displacement, 0, PI / 2, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else if (gamepad1.dpad_up) {
                robot.setTargetPoint(0, displacement, PI / 2, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else if (gamepad1.dpad_down) {
                robot.setTargetPoint(0, -displacement, PI / 2, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else if (gamepad1.left_bumper) {
                robot.setTargetPoint(0, 0, PI, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else if (gamepad1.right_bumper) {
                robot.setTargetPoint(0, 0, 0, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else if (gamepad1.a) {
                robot.setTargetPoint(targetX, targetY, targetTheta, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
//            } else if (gamepad1.b) {
//                robot.setTargetPoint(111, 63, 1.91, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else {
                robot.drivetrain.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            }

            robot.update();
        }
    }
}
