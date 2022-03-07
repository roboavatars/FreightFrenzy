package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;
import static java.lang.Math.PI;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Turret;

@TeleOp
public class TurretPDTuning extends LinearOpMode {
    public static boolean enabled = true;
    public static double targetTheta = 0.5;

    @Override
    public void runOpMode() {
        Turret deposit = new Turret(this, false, PI/2);

        waitForStart();

        while (opModeIsActive()) {
            if (enabled) {
                deposit.setDepositing(targetTheta * PI);
            } else {
                deposit.setPower(0);
            }

            addPacket("current theta", deposit.getTheta());
            addPacket("target theta", targetTheta * PI);
            sendPacket();
        }
    }
}
