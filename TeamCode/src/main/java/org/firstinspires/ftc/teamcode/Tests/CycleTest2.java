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

@TeleOp(name = "0 0 0 0 0 op teleop (cycle test 2)")
@Config
public class CycleTest2 extends LinearOpMode {
    public static boolean enabled = true;
    public static double lockTheta = 0.5;
    public static double turretMovingAngle = 0.25;

    public static double pArmGoingUp = Deposit.pArmGoingUp;
    public static double pArmGoingDown = Deposit.pArmGoingDown;

    public static double pSlidesExtend = 35;
    public static double pSlidesRetract = 20;

    public static int slidesExtendDist = 25;

    private boolean depositToggle = false;
    private int depositGatePos = 0;

    @Override
    public void runOpMode() {
        Deposit deposit = new Deposit(this, false, PI/2);
        Drivetrain dt = new Drivetrain(this, 0, 0, PI/2);
        Intake intake = new Intake(this, false);

        waitForStart();

        while (opModeIsActive()) {
            dt.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            dt.updatePose();

            if (gamepad1.start) {
                lockTheta = turretMovingAngle;
            }

            if (gamepad1.right_bumper) {
                intake.extend();
                intake.on();
                intake.flipDown();
            } else if (gamepad1.left_bumper) {
                intake.home();
                intake.flipUp();
                intake.off();
            } else if (gamepad1.a) {
                deposit.open();
                intake.reverse();
            } else {
                intake.off();
            }

            if (gamepad1.x) {
                deposit.hold();

                deposit.setArmPIDCoefficients(pArmGoingUp);
                deposit.setArmControls(Constants.DEPOSIT_ARM_HIGH);

                deposit.setSlidesPIDCoefficients(pSlidesExtend);
                deposit.setSlidesControls(slidesExtendDist * (int) Constants.DEPOSIT_SLIDES_TICKS_PER_INCH);
            } else if (gamepad1.y) {
                deposit.open();
            } else if (gamepad1.b) {
                deposit.setArmPIDCoefficients(pArmGoingDown);
                deposit.setArmControls(Constants.DEPOSIT_ARM_HOME);

                deposit.setSlidesPIDCoefficients(pSlidesRetract);
                deposit.setSlidesControls(0);
            }

//            if (enabled) {
//                targetTheta = (lockTheta * PI - dt.theta + PI/2) % (2 * PI);
//                if (targetTheta < 0) {
//                    targetTheta += 2 * PI;
//                }
//                // prevents wrap from 0 to 2pi from screwing things up
//                // now wrap is from -pi/2 to 3pi/2 (which the turret will never reach)
//                if (targetTheta > 3*PI/2) {
//                    targetTheta -= 2*PI;
//                }
////                deposit.setTurretTheta(targetTheta);
//                deposit.setTurretThetaFF(targetTheta, dt.commandedW);
//            } else {
//                deposit.setTurretPower(0);
//            }

            addPacket("1 arm ticks", deposit.getArmPosition());
            addPacket("2 slide inches", deposit.getSlidesDistInches());
            addPacket("3 slide ticks", deposit.getSlidesPosition());
            addPacket("4 slides target ticks", deposit.targetSlidesTicks);
            addPacket("5 current theta", deposit.getTurretTheta());
            sendPacket();
        }
    }
}
