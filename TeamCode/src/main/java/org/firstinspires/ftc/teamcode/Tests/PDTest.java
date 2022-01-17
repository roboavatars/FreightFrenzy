package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawDrivetrain;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;
import static java.lang.Math.PI;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Drivetrain;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@TeleOp(name = "PD Controller Test")
@Config
public class PDTest extends LinearOpMode {
    public static double targetX = 0;
    public static double targetY = 0;
    public static double targetTheta = 0;

    public static double xKp = 0.8;
    public static double yKp = 1;
    public static double thetaKp = 0.1;
    public static double xKd = 0.55;
    public static double yKd = 0.0023;
    public static double thetaKd = 8;

    public double x, y, theta, vx, vy, w;
    private double prevX, prevY, prevTheta, prevVx, prevVy, prevW, prevTime;

    @Override
    public void runOpMode() {

        Drivetrain drivetrain = new Drivetrain(this, 0, 0, 0);

        waitForStart();

        while(opModeIsActive()) {
            /*
            if (gamepad1.x) {
                robot.resetOdo(111, 63, PI/2);
            }

            if (gamepad1.dpad_left) {
                robot.setTargetPoint(71, 72, PI/2, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else if (gamepad1.dpad_right) {
                robot.setTargetPoint(121, 72, PI/2, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else if (gamepad1.dpad_up) {
                robot.setTargetPoint(96, 122, PI/2, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else if (gamepad1.dpad_down) {
                robot.setTargetPoint(96, 22, PI/2, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else if (gamepad1.left_bumper) {
                robot.setTargetPoint(96, 72, PI, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else if (gamepad1.right_bumper) {
                robot.setTargetPoint(96, 72, 0, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else if (gamepad1.a) {
                robot.setTargetPoint(targetX, targetY, targetTheta, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else if (gamepad1.b) {
                robot.setTargetPoint(111, 63, 1.91, 0, 0, 0, xKp, yKp, thetaKp, xKd, yKd, thetaKd);
            } else {
                robot.drivetrain.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            }
             */

            drivetrain.updatePose();

            double curTime = System.currentTimeMillis();
            double timeDiff = curTime / 1000 - prevTime;
            x = drivetrain.x;
            y = drivetrain.y;
            theta = drivetrain.theta;
            vx = (x - prevX) / timeDiff; vy = (y - prevY) / timeDiff; w = (theta - prevTheta) / timeDiff;

            prevX = x; prevY = y;
            prevTheta = theta;
            prevTime = curTime / 1000;
            prevVx = vx; prevVy = vy; prevW = w;

            targetTheta = targetTheta % (2*PI);
            if (targetTheta < 0) {
                targetTheta += 2*PI;
            }

            // Picking the Smaller Distance to Rotate
            double thetaControl;
            if (abs(theta - targetTheta) > PI) {
                thetaControl = abs(theta - targetTheta) / (theta - targetTheta) * (abs(theta - targetTheta) - 2*PI);
            } else {
                thetaControl = theta - targetTheta;
            }

            drawDrivetrain(targetX, targetY, targetTheta, "blue");
            drawDrivetrain(x, y, theta, "green");

            drivetrain.setGlobalControls(xKp * (targetX - x) + xKd * (0 - vx), yKp * (targetY - y) + yKd * (0 - vy), thetaKp * (targetTheta-thetaControl) + thetaKd * (0 - w));

            addPacket("1 x", x);
            addPacket("1 y", y);
            addPacket("1 theta", theta);

            addPacket("x P", xKp * (targetX - x));
            addPacket("x D", xKd * - vx);
            addPacket("y P", yKp * (targetY - y));
            addPacket("y D", yKd * - vy);
            addPacket("omega P", thetaKp * (targetTheta-thetaControl));
            addPacket("omega D", thetaKd * - w);
//            addPacket("vx", robot.vx);
//            addPacket("vy", robot.vy);
//            addPacket("w", robot.w);
            sendPacket();
        }
    }
}