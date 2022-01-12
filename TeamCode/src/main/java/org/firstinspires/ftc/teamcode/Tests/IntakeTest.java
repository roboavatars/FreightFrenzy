package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Pathing.Ramsete.PDController;
import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Intake;

@TeleOp
@Config
public class IntakeTest extends LinearOpMode {
    public static boolean intaking = false;

    private boolean firstReturnLoop = false;
    private double curTime;
    private double intakeGetHomeTime;

    @Override
    public void runOpMode() {
        Intake intake = new Intake(this, false);

        waitForStart();

        while (opModeIsActive()) {
            curTime = System.currentTimeMillis();
            if (intaking) {
                intake.extend();
                intake.on();
                intake.down();
                intake.checkIfStalling();
                firstReturnLoop = false;
            } else {
                intake.retract();
                intake.up();
                if (!firstReturnLoop){
                    intakeGetHomeTime = -1;
                    firstReturnLoop = true;
                }
                if (intakeGetHomeTime == -1 && intake.slidesIsHome()) {
                    intakeGetHomeTime = curTime;
                    intake.reverse();
                } else if (intakeGetHomeTime != -1 && curTime - intakeGetHomeTime < Constants.TRANSFER_TIME) {
                    intake.reverse();
                } else {
                    intake.off();
                }

                addPacket("slides ticks", intake.getSlidesTicks());
                addPacket("intake get home time", intakeGetHomeTime);
                addPacket("current time", curTime);
                sendPacket();

            }
        }
    }
}
