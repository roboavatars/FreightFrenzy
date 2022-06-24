package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Deposit;
import org.firstinspires.ftc.teamcode.RobotClasses.Intake;

@TeleOp
@Config
public class ResetEncoders extends LinearOpMode {
    @Override
    public void runOpMode() {
        Deposit deposit = new Deposit(this, false, true, Constants.ARM_AUTO_INIT_POS);
        Intake intake = new Intake(this, false, true);
    }

}
