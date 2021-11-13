package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RobotClasses.Drivetrain;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawDrivetrain;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawField;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

@TeleOp(name = "Odometry / Drivetrain Test")
public class OdoDrivetrainTest extends LinearOpMode {
    private double x, y, theta, prevTime;

    @Override
    public void runOpMode() {
        Drivetrain dt = new Drivetrain(this, 54.5, 15, PI/2);

        waitForStart();

        while(opModeIsActive()) {
            dt.setControls(-gamepad1.left_stick_y, -0.5 * gamepad1.right_stick_x);

            if (gamepad1.x) {
                dt.resetOdo(54.5, 15, PI/2);
            }

            if (gamepad1.dpad_right || gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left) {
                dt.setRawPower(gamepad1.dpad_right ? 0.5 : 0, gamepad1.dpad_up ? 0.5 : 0, gamepad1.dpad_down ? 0.5 : 0, gamepad1.dpad_left ? 0.5 : 0);
            }

            dt.updatePose();
            x = dt.x;
            y = dt.y;
            theta = dt.theta;

            double curTime = (double) System.currentTimeMillis() / 1000;
            double timeDiff = curTime - prevTime;
            prevTime = curTime;

            drawField();
            drawDrivetrain(x, y, theta, "green");
            addPacket("X", x);
            addPacket("Y", y);
            addPacket("Theta", theta);
            addPacket("Update Frequency (Hz)", 1 / timeDiff);
            addPacket("Positions", dt.motorFrontLeft.getCurrentPosition() + " " + dt.motorFrontRight.getCurrentPosition() + " " + dt.motorBackLeft.getCurrentPosition() + " " + dt.motorBackRight.getCurrentPosition());
            addPacket("podL", dt.podL);
            addPacket("podR", dt.podR);
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