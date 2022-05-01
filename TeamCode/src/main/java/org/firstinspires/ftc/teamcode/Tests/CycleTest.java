package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Deposit;
import org.firstinspires.ftc.teamcode.RobotClasses.Intake;

//Config in Constants.java
@TeleOp(name = "Cycle Test")
@Config
public class CycleTest extends LinearOpMode {
    public static boolean extend = false;

    int depositCase = 0;
    private double lastTime;
    private int intakeCase = 1;
    public static double transferThreshold = 1000;

    @Override
    public void runOpMode() throws InterruptedException {
        Deposit deposit = new Deposit(this,false, 0, 0);
        Intake intake = new Intake(this,false);

        intake.flipDown();
        intake.home();
        intake.off();

        waitForStart();
        while (opModeIsActive()) {
            //intake states
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
                    intake.setPower(-.25);
                    intake.flipUp();
                    if (intake.slidesIsHome() && deposit.slidesisHome()) {
                        intakeCase ++;
                        lastTime = System.currentTimeMillis();
                    }
                    break;
                case 4:
                    intake.reverse();
                    if (System.currentTimeMillis() - lastTime > transferThreshold) {
                        intakeCase = 1;
                        depositCase = 2;
                    }
                    break;
            }

            //deposit states
            switch (depositCase) {
                case 1:
                    deposit.retractSlides();
                    deposit.armHome();
                    break;
                case 2:
                    deposit.extendSlides();
                    deposit.armOut();
                    if (gamepad1.a) {
                        depositCase = 1;
                    }
            }
        }
    }
}
