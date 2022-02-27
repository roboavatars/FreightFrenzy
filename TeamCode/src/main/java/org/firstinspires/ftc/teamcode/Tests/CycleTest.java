package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Deposit;
import org.firstinspires.ftc.teamcode.RobotClasses.Drivetrain;
import org.firstinspires.ftc.teamcode.RobotClasses.Intake;
import org.firstinspires.ftc.teamcode.RobotClasses.Turret;

@TeleOp(name = "0 cycle test")
@Config
public class CycleTest extends LinearOpMode {

    public static double lockTheta = 0.5;
    public static double turretMovingAngle = 0.2;

    public static int slidesExtendDist = 16;
    public static int armPos = Constants.DEPOSIT_ARM_MID;

    private boolean home = true;
    private boolean homeToggle = false;
    private boolean intakeHome;

    private double startRetractTime = 0;

    @Override
    public void runOpMode() {
        Turret turret = new Turret(this, false, PI/2);
        Deposit deposit = new Deposit(this, false);
        Drivetrain dt = new Drivetrain(this, 0, 0, PI/2);
        Intake intake = new Intake(this, false);

        waitForStart();

        while (opModeIsActive()) {
            dt.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            dt.updatePose();

            /*if (gamepad1.right_bumper) {
                intake.extend();
                intake.on();
                intake.flipDown();
                startRetractTime = System.currentTimeMillis();
                intakeHome = false;
            } else if (gamepad1.left_bumper) {
                intake.home();
                intake.flipUp();
                intakeHome = true;
            } else if (gamepad1.a) {
                deposit.open();
                intake.reverse();
                startRetractTime = System.currentTimeMillis();
                intakeHome = false;
            } else {
                intake.off();
                startRetractTime = System.currentTimeMillis();
                intakeHome = false;
            }

            if (intakeHome) {
                if (System.currentTimeMillis() < startRetractTime + 1000) {
                    intake.on();
                } else {
                    intake.off();
                }
            }*/

            if (gamepad1.x && !homeToggle) {
                homeToggle = true;
                if (home) {
                    deposit.hold();
                }
                home = !home;
            } else if (!gamepad1.x && homeToggle) {
                homeToggle = false;
            }

            if (gamepad1.y) {
                deposit.open();
            }

            if (home) {
//                turret.setDepositing(PI/2);

                deposit.setArmPIDCoefficients(Deposit.pArmDown, Deposit.dArmDown);
                deposit.setArmTarget(Constants.DEPOSIT_ARM_HOME);

                deposit.setSlidesTarget(0);
            } else {
//                turret.setDepositing(turretMovingAngle * PI);

                deposit.setArmPIDCoefficients(Deposit.pArmUp, Deposit.dArmUp);
//                if (!deposit.slidesAtPosPercent(0.75)) {
//                    deposit.setArmTarget(Constants.DEPOSIT_ARM_MIDWAY);
//                } else {
                    deposit.setArmTarget(armPos/*Constants.DEPOSIT_ARM_HIGH*/);
//                }

                deposit.setSlidesTarget((int) (slidesExtendDist * Deposit.DEPOSIT_SLIDES_TICKS_PER_INCH));
            }
            deposit.setSlidesPIDCoefficients(Deposit.pSlides);
//            deposit.setSlidesControls();
            deposit.setArmControls();

            addPacket("1 arm ticks", deposit.getArmPosition());
            addPacket("1 arm angle", deposit.getArmAngle());
            addPacket("1 arm error", deposit.getArmError());
            addPacket("2 slide inches", deposit.getSlidesDistInches());
            addPacket("3 slide ticks", deposit.getSlidesPosition());
            addPacket("4 slides target ticks", deposit.targetSlidesTicks);
            addPacket("5 current theta", turret.getTurretTheta());
            sendPacket();
        }
    }
}
