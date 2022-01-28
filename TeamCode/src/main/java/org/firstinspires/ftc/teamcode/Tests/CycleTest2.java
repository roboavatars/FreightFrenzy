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
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@TeleOp(name = "0 0 cycle test 2")
@Config
public class CycleTest2 extends LinearOpMode {
    public static boolean enabled = true;
    public static double lockTheta = 0.5;

    public static int slidesExtendDist = 0;

    private boolean armHome = true;
    private boolean slidesHome = true;
    private boolean depositToggle = false;
    private int depositGatePos = 0;

    @Override
    public void runOpMode() {
        Deposit deposit = new Deposit(this, false, PI/2);
        Drivetrain dt = new Drivetrain(this, 0, 0, PI/2);
        Intake intake = new Intake(this, false);


        waitForStart();

        double targetTheta = 0;

        while (opModeIsActive()) {
            dt.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            dt.updatePose();

            if (gamepad1.a) {
                intake.extend();
                intake.on();
                intake.flipDown();
            } else if (gamepad1.right_trigger > .1){
                intake.reverse();
            } else {
                intake.home();
                intake.flipUp();
                intake.off();
            }


            if (gamepad1.dpad_up){
                armHome = false;
            } else if (gamepad1.dpad_down){
                armHome = true;
            }

            if (gamepad1.dpad_right){
                slidesHome = false;
            } else if (gamepad1.dpad_left){
                slidesHome = true;
            }

            if (armHome){
                deposit.setArmControls(Constants.DEPOSIT_ARM_HOME);
            } else {
                deposit.setArmControls(Constants.DEPOSIT_ARM_HIGH);
            }

            if (slidesHome){
                deposit.setSlidesControls(0);
            } else {
                deposit.setSlidesControls(slidesExtendDist * (int) Constants.DEPOSIT_SLIDES_TICKS_PER_INCH);
            }

            if (enabled) {
                targetTheta = (lockTheta * PI - dt.theta + PI/2) % (2 * PI);
                if (targetTheta < 0) {
                    targetTheta += 2 * PI;
                }
                // prevents wrap from 0 to 2pi from screwing things up
                // now wrap is from -pi/2 to 3pi/2 (which the turret will never reach)
                if (targetTheta > 3*PI/2) {
                    targetTheta -= 2*PI;
                }
//                deposit.setTurretTheta(targetTheta);
                deposit.setTurretThetaFF(targetTheta, dt.commandedW);
            } else {
                deposit.setTurretPower(0);
            }

            addPacket("1 arm ticks", deposit.getArmPosition());
            addPacket("2 slide inches", deposit.getSlidesDistInches());
            addPacket("2 slide ticks", deposit.getSlidesPosition());
            addPacket("arm home", armHome);
            addPacket("slide home", slidesHome);
            addPacket("5 current theta", deposit.getTurretTheta());
            sendPacket();
        }
    }
}
