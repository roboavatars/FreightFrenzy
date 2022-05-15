package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotClasses.Deposit;
import org.firstinspires.ftc.teamcode.RobotClasses.Drivetrain;
import org.firstinspires.ftc.teamcode.RobotClasses.Intake;

import java.util.List;

//Config in Constants.java
@TeleOp(name = "Cycle Test")
@Config
public class CycleTest extends LinearOpMode {
    public static boolean extend = false;
    public static boolean shared = false;

    int depositCase = 0;
    int sharedCase = 0;
    private double transferStart;
    private double sharedDepositStart;
    private double sharedRetractStart;
    private int intakeCase = 1;
    public static double transferThreshold = 500;
    public static double turretDepositThreshold = 1000;
    public static double turretHomeThreshold = 1000;

    @Override
    public void runOpMode() throws InterruptedException {
        Deposit deposit = new Deposit(this,false, 0, 0);
        Intake intake = new Intake(this,false);
        Drivetrain dt = new Drivetrain(this, 0,0,0);

        intake.flipDown();
        intake.home();
        intake.off();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        waitForStart();
        while (opModeIsActive()) {
            int i = 1;
            for (LynxModule hub : allHubs) {
                addPacket("hub " + i, hub.getCurrent(CurrentUnit.AMPS));
                i++;
            }
            sendPacket();

            dt.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

            intake.updateSlides();
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
                    if (gamepad1.right_trigger < .1) intakeCase++;
                    break;
                case 3:
                    intake.home();
                    intake.setPower(.5);
                    intake.flipUp();
                    if (intake.slidesIsHome() && deposit.slidesisHome()) {
                        intakeCase++;
                        transferStart = System.currentTimeMillis();
                    }
                    break;
                case 4:
                    intake.reverse();
                    if (System.currentTimeMillis() - transferStart > transferThreshold) {
                        intakeCase = 1;
                        depositCase = 2;
                        sharedCase = 2;
                    }
                    break;
            }

            //deposit states
            if (shared) {
                switch (sharedCase) {
                    case 1:
                        deposit.armHome();
                        sharedDepositStart = System.currentTimeMillis();
                        break;
                    case 2:
                        deposit.armOut();
                        if (System.currentTimeMillis() - sharedDepositStart > turretDepositThreshold) sharedCase ++;
                        break;
                    case 3:
//                        deposit.turretRight();
                        sharedRetractStart = System.currentTimeMillis();
                        if (gamepad1.a) {
                            depositCase++;
                        }
                        break;
                    case 4:
//                        deposit.turretHome();
                        if (System.currentTimeMillis() - sharedRetractStart > turretHomeThreshold) {
                            depositCase = 1;
                        }
                }
            } else {
//                deposit.turretHome();
                switch (depositCase) {
                    case 1:
                        deposit.retractSlides();
                        deposit.armHome();
                        deposit.release();
                        break;
                    case 2:
                        deposit.extendSlides();
                        deposit.armOut();
                        deposit.hold();
                        if (gamepad1.a) {
                            depositCase = 1;
                        }
                        break;
                }
            }
        }
    }
}
