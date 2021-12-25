package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Drivetrain;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawDrivetrain;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawField;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

@TeleOp(name = "Combined Localization Test")
@Disabled
public class LocalizationTest extends LinearOpMode {
    private double odoX, odoY, odoTheta;
    private double odoVx, odoVy, odoW;
    private double camX, camY, camTheta;
    private double x, y, theta;
    private double startTime, runTime, curTime, prevTime, timeDiff;

    private double startX = 111;
    private double startY = 63;
    private double startTheta = PI/2;

    @Override
    public void runOpMode() {
        Drivetrain dt = new Drivetrain(this, startX, startY, startTheta);

        waitForStart();
        startTime = System.currentTimeMillis();

        while(opModeIsActive()) {
            // Driving
            dt.setControls(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);

            // Update Time Variables
            curTime = (double) System.currentTimeMillis() / 1000;
            timeDiff = curTime - prevTime;
            runTime = curTime - startTime / 1000;
            prevTime = curTime;

            // Update Drivetrain Kinematics Variables
            dt.updatePose();

            odoVx = (dt.x - odoX) / timeDiff;
            odoVy = (dt.y - odoY) / timeDiff;
            odoW = (dt.theta - odoTheta) / timeDiff;

            odoX = dt.x;
            odoY = dt.y;
            odoTheta = dt.theta;

            // Dashboard
            drawField();
            drawDrivetrain(odoX, odoY, odoTheta, "blue");
            drawDrivetrain(x, y, theta, "black");
            addPacket("X", x);
            addPacket("Y", y);
            addPacket("Theta", theta);
            addPacket("Run Time", runTime);
            addPacket("Update Frequency (Hz)", 1 / timeDiff);
            sendPacket();
        }

    }
}