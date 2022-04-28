package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Intake;

//Config in Constants.java
@TeleOp(name = "Intake Test")
@Config
public class IntakeTest extends LinearOpMode {
    private double lastTime;
    private int intakeCase = 1;
    public static double transferThreshold = 1000;

    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake(this,false);

        intake.flipDown();
        intake.home();
        intake.off();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.right_trigger > .1) {
                intakeCase = 2;
            }

            switch (intakeCase) {
                case 1:
                    intake.flipDown();
                    intake.off();
                    break;
                case 2:
                    intake.extend();
                    intake.on();
                    intake.flipDown();
                    if (gamepad1.right_trigger < .1) intakeCase ++;
                    break;
                case 3:
                    intake.home();
                    intake.off();
                    intake.flipUp();
                    if (intake.slidesIsHome()) {
                        intakeCase ++;
                        lastTime = System.currentTimeMillis();
                    }
                    break;
                case 4:
                    intake.reverse();
                    if (System.currentTimeMillis() - lastTime > transferThreshold) intakeCase = 1;
                    break;
            }
        }
    }
}
