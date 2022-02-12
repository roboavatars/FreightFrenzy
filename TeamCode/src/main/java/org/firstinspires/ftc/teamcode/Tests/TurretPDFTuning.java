package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Drivetrain;
import org.firstinspires.ftc.teamcode.RobotClasses.Turret;

@TeleOp(name = "0 turret tuning")
@Config
public class TurretPDFTuning extends LinearOpMode {
    public static boolean enabled = true;
    public static double lockTheta = 0.5;

    @Override
    public void runOpMode() {
        Turret deposit = new Turret(this, false, PI/2);
        Drivetrain dt = new Drivetrain(this, 0, 0, PI/2);

        waitForStart();

        double targetTheta = 0;

        while (opModeIsActive()) {
            dt.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            dt.updatePose();

            if (enabled) {
                targetTheta = (lockTheta * PI - dt.theta + PI/2) % (2 * PI);
                if (targetTheta < 0) {
                    targetTheta += 2 * PI;
                }
                // prevents wrap from 0 to 2pi from screwing things up
                // now wrap is from -pi/2 to 3pi/2 (which the turret will never reach)
                if (targetTheta > 3*PI/2) {
                    targetTheta -= 2*PI;
                }
//                deposit.setTurretTheta(targetTheta);
                deposit.setTurretThetaFF(targetTheta, dt.commandedW);
            } else {
                deposit.setTurretPower(0);
            }

            addPacket("1 dt theta", dt.theta);
            addPacket("2 lock theta", lockTheta * PI);
            addPacket("3 global current theta", dt.theta + deposit.getTurretTheta() - PI/2);
            addPacket("4 current theta", deposit.getTurretTheta());
            addPacket("5 target theta", targetTheta);
            sendPacket();
        }
    }
}
