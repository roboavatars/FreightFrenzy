package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Drivetrain;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawDrivetrain;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawField;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

@TeleOp(name = "PD Controller Test")
@Config
public class PDTest extends LinearOpMode {
    private Drivetrain dt;
    public static double targetY = 72;
    public static double targetTheta = PI/2;

    public static double yKp = 0.035;
    public static double thetaKp = 0;
    public static double yKd = 0.01;
    public static double thetaKd = 0;

    @Override
    public void runOpMode() {
        dt = new Drivetrain(this, 54.5, 39, PI/2);

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.x) {
                dt.resetOdo(54.5, 39, PI/2);
            }

            targetTheta = targetTheta % (2*PI);
            if (targetTheta < 0) {
                targetTheta += 2*PI;
            }

            // Picking the Smaller Distance to Rotate
            double thetaControl;
            if (Math.abs(dt.theta - targetTheta) > PI) {
                thetaControl = Math.abs(dt.theta - targetTheta) / (dt.theta - targetTheta) * (Math.abs(dt.theta - targetTheta) - 2*PI);
            } else {
                thetaControl = dt.theta - targetTheta;
            }

            dt.setControls(yKp * (targetY - dt.y), thetaKp * (-thetaControl));

//            if (gamepad1.dpad_left) {
//                dt.setControls(yKp * (72 - dt.y), thetaKp * (PI/2 - dt.theta));
//            } else if (gamepad1.dpad_right) {
//                dt.setControls(yKp * (72 - dt.y), thetaKp * (PI/2 - dt.theta));
//            } else if (gamepad1.dpad_up) {
//                dt.setControls(yKp * (122 - dt.y), thetaKp * (PI/2 - dt.theta));
//            } else if (gamepad1.dpad_down) {
//                dt.setControls(yKp * (22 - dt.y), thetaKp * (PI/2 - dt.theta));
//            } else if (gamepad1.left_bumper) {
//                dt.setControls(yKp * (72 - dt.y), thetaKp * (PI - dt.theta));
//            } else if (gamepad1.right_bumper) {
//                dt.setControls(yKp * (72 - dt.y), thetaKp * (-dt.theta));
//            } else if (gamepad1.a) {
//                dt.setControls(yKp * (72 - dt.y), thetaKp * (PI/2 - dt.theta));
//            } else if (gamepad1.b) {
//                dt.setControls(yKp * (63 - dt.y), thetaKp * (PI/2 - dt.theta));
//            } else {
//                dt.setControls(-gamepad1.left_stick_y, -gamepad1.right_stick_x);
//            }

            dt.updatePose();

            drawField();
            drawDrivetrain(54.5, dt.y, dt.theta, "green");
            addPacket("x", dt.x);
            addPacket("y", dt.y);
            addPacket("theta", dt.theta);
            sendPacket();
        }

        dt.stop();
    }
}
