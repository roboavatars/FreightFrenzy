package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Deposit;
import org.firstinspires.ftc.teamcode.RobotClasses.Drivetrain;

//Tune constants Kp, Kd, and Kf through dashboard in the Deposit class config

@TeleOp
@Config
public class TurretPDFTuning extends LinearOpMode {
    public static double initialTheta = 0;
    double lockTheta;
    double targetTheta;

    @Override
    public void runOpMode() {
        Deposit deposit = new Deposit(this, false);
        Drivetrain dt = new Drivetrain(this, 0, 0, PI / 2);


        waitForStart();

        while (opModeIsActive()) {
            dt.setControls(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
            dt.updatePose();

            lockTheta = Math.atan2(-dt.y, -dt.x);
            targetTheta = (lockTheta - dt.theta) % (2 * PI);
            if (targetTheta < 0) {
                targetTheta += 2 * PI;
            }
            deposit.setTurretThetaPDFF(targetTheta, dt.commandedW);

            addPacket("lock theta", lockTheta);
            addPacket("target theta", targetTheta);
            addPacket("theta", deposit.getTurretTheta());
            addPacket("error", deposit.getTurretError());
            sendPacket();

        }
    }
}
