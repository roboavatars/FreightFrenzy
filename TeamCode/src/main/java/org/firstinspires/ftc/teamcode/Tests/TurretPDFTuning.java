package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;
import static java.lang.Math.PI;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Drivetrain;

@TeleOp(name = "turret tuning")
public class TurretPDFTuning extends LinearOpMode {
    public static boolean enabled = true;
    public static double lockTheta = 0.5;

    @Override
    public void runOpMode() {
        Turret turret = new Turret(this, false, PI / 2);
        Drivetrain dt = new Drivetrain(this, 0, 0, PI / 2);

        waitForStart();

        double targetTheta = 0;

        while (opModeIsActive()) {
            dt.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            dt.updatePose();

            if (enabled) {
                targetTheta = (Math.atan2(30 - dt.y, dt.x) - dt.theta + PI / 2) % (2 * PI);
                if (targetTheta < 0) {
                    targetTheta += 2 * PI;
                }
                // prevents wrap from 0 to 2pi from screwing things up
                // now wrap is from -pi/2 to 3pi/2 (which the turret will never reach)
                if (targetTheta > 3 * PI / 2) {
                    targetTheta -= 2 * PI;
                }
//                deposit.setTurretTheta(targetTheta);
                turret.setTurretThetaFF(targetTheta, dt.commandedW);
            } else {
                turret.setPower(0);
            }

            addPacket("1 dt theta", dt.theta);
            addPacket("2 lock theta", lockTheta * PI);
            addPacket("3 global current theta", dt.theta + turret.getTheta() - PI / 2);
            addPacket("4 current theta", turret.getTheta());
            addPacket("5 target theta", targetTheta);
            sendPacket();
        }
    }
}
