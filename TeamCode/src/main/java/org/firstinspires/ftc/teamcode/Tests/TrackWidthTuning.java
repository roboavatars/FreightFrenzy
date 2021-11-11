package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawDrivetrain;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawField;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Drivetrain;

@TeleOp(name = "Track width tuning")
@Config
public class TrackWidthTuning extends LinearOpMode {
    public static double rotations = 3;

    private double x, y, theta, prevTime, prevTheta, adjustedTheta, cumulativeTheta;

    @Override
    public void runOpMode() {
        Drivetrain dt = new Drivetrain(this, 90, 9, PI/2);

        waitForStart();

        while(opModeIsActive()) {
            dt.setControls(0, Math.abs(gamepad1.right_stick_x));

            if (gamepad1.x) {
                dt.resetOdo(90, 9, PI/2);
            }

            if (gamepad1.dpad_right || gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left) {
                dt.setRawPower(gamepad1.dpad_right ? 0.5 : 0, gamepad1.dpad_up ? 0.5 : 0, gamepad1.dpad_down ? 0.5 : 0, gamepad1.dpad_left ? 0.5 : 0);
            }

            prevTheta = theta;

            dt.updatePose();
            x = dt.x;
            y = dt.y;
            theta = dt.theta - PI/2;


            double curTime = (double) System.currentTimeMillis() / 1000;
            double timeDiff = curTime - prevTime;
            prevTime = curTime;

            double trackWidth = dt.ODOMETRY_TRACK_WIDTH * theta/ (rotations * 2*PI);

            drawField();
            drawDrivetrain(x, y, theta, "green");
            addPacket("X", x);
            addPacket("Y", y);
            addPacket("Theta", theta);
            addPacket("Track Width", trackWidth);
            addPacket("Update Frequency (Hz)", 1 / timeDiff);
            addPacket("podR", dt.podR);
            addPacket("podL", dt.podL);
            addPacket("R zeros", dt.zeroR);
            addPacket("L zeros", dt.zeroL);
            sendPacket();

            telemetry.addData("X: ", x);
            telemetry.addData("Y: ", y);
            telemetry.addData("Theta: ", theta);
            telemetry.update();

        }
    }
}