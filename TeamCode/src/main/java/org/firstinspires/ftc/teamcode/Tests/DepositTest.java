package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Deposit;

@TeleOp
@Config
public class DepositTest extends LinearOpMode {
    public static boolean home = true;
    public static double slidesInches = 0;
    public static double theta = 0;

    @Override
    public void runOpMode(){
        Deposit deposit = new Deposit(this, false);
        waitForStart();
        while (opModeIsActive()) {
            if (home) {
                deposit.setControlsHome();
            } else {
                deposit.setControlsDepositing(theta, Constants.DEPOSIT_ARM_TOP_GOAL, slidesInches);
            }
            deposit.update(0,0);
        }
    }
}
