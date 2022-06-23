package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Deposit;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@TeleOp
@Config
public class ArmTest extends LinearOpMode {
    public static int pos = 0;
    public static int initPos = 0;

    @Override
    public void runOpMode() {
        Deposit deposit = new Deposit(this, false, initPos);

        waitForStart();

        while (opModeIsActive()) {
            deposit.setArmControls(pos);
            deposit.updateArm(Robot.DepositTarget.high);
            addPacket("arm pos", deposit.getArmPos());
            addPacket("arm theta", deposit.getArmTheta());
            sendPacket();
        }
    }

}
