package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Deposit;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@TeleOp
@Config
public class DepositTest extends LinearOpMode {
    public static boolean home = true;
    public static double slidesInches = 0;

    @Override
    public void runOpMode(){
        Deposit deposit = new Deposit(this, false);
        waitForStart();
        while (opModeIsActive()) {
            if (home) {
                deposit.setDepositHome();
            } else {
                deposit.setDepositControls(Robot.DepositTarget.allianceHigh, slidesInches);
            }
            deposit.update();
        }
    }
}
