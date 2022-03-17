package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Arm;

@TeleOp
@Config
public class ArmGravityPIDTest extends LinearOpMode {

    public static boolean depositClose = true;
    public static boolean armUp = false;
    public static int armUpTarget = 500;
    public static double pUp = 0.003;
    public static double pDown = 0.0055;
    public static double dUp = 0.0015;
    public static double dDown = 0.002;
    public static double f = 0.1;

    @Override
    public void runOpMode() {
        Arm deposit = new Arm(this, false);

        waitForStart();
        while (opModeIsActive()) {
            Arm.pArm = armUp ? pUp : pDown;
            Arm.dArm = armUp ? dUp : dDown;
            Arm.fGravity = f;
            if (depositClose) deposit.hold();
            else deposit.open();

            deposit.targetArmPos = armUp ? armUpTarget : 0;
            deposit.update();

            addPacket("arm ticks", deposit.getArmPosition());
            addPacket("arm theta", deposit.getArmAngle());
            sendPacket();
        }
    }
}
