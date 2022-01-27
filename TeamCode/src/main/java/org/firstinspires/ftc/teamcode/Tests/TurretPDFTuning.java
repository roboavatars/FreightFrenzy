package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Deposit;
import org.firstinspires.ftc.teamcode.RobotClasses.Drivetrain;

@TeleOp(name = "0 0 turret tuning")
@Config
public class TurretPDFTuning extends LinearOpMode {
    public static boolean enabled = true;
    public static double lockTheta = 0.5;

    @Override
    public void runOpMode() {
        Deposit deposit = new Deposit(this, false);
        Drivetrain dt = new Drivetrain(this, 0, 0, PI/2);

        waitForStart();

        double targetTheta = 0;

        while (opModeIsActive()) {
            dt.setControls(0, 0, -gamepad1.right_stick_x);
            dt.updatePose();

            if (enabled) {
                targetTheta = (lockTheta * PI - dt.theta + PI/2) % (2 * PI);
                if (targetTheta < 0) {
                    targetTheta += 2 * PI;
                }
                deposit.setTurretTheta(targetTheta);
//                deposit.setTurretThetaFF(targetTheta, dt.commandedW);
            } else {
                deposit.setTurretPower(0);
            }

            addPacket("dt theta", dt.theta);
            addPacket("lock theta", lockTheta * PI);
            addPacket("global current theta", dt.theta + deposit.getTurretTheta() - PI/2);
            addPacket("current theta", deposit.getTurretTheta());
            addPacket("target theta", targetTheta);
            sendPacket();
        }
    }
}
