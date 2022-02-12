package org.firstinspires.ftc.teamcode.Tests;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Deposit;

@TeleOp
@Config
public class DepositTest extends LinearOpMode {
    public static boolean home = true;
    public static double slidesInches = 0;

    @Override
    public void runOpMode(){
        Deposit deposit = new Deposit(this, false, PI/2);
        waitForStart();
        while (opModeIsActive()) {
            if (home) {
                deposit.setDepositHome();
            } else {
                deposit.setDepositControls(Constants.DEPOSIT_ARM_HIGH, slidesInches);
            }
            deposit.update();
        }
    }
}
