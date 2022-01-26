package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Deposit;
import org.firstinspires.ftc.teamcode.RobotClasses.Intake;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@TeleOp
@Config
public class CycleTest extends LinearOpMode {
    public static int slidesExtendDist = 0;
    public static double turretTheta = 0;

    private boolean armHome = true;
    private boolean slidesHome = true;
    private boolean depositToggle = false;
    private int depositGatePos = 0;


    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, 0, 0, 0, false, false);
        robot.intake.home();
        robot.intake.flipUp();
        robot.intake.off();
        robot.deposit.open();


        waitForStart();

        while (opModeIsActive()) {
            addPacket("arm pos", robot.deposit.getArmPosition());
            addPacket("turret theta", robot.deposit.getTurretTheta());
            addPacket("slides pos", robot.deposit.getSlidesPosition());
            sendPacket();

            if (gamepad1.a) {
                robot.intake.extend();
                robot.intake.on();
                robot.intake.flipDown();
            } else if (gamepad1.right_trigger > .1){
                robot.intake.reverse();
            } else {
                robot.intake.home();
                robot.intake.flipUp();
                robot.intake.off();
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
                robot.deposit.setArmControls(Constants.DEPOSIT_ARM_HOME);
            } else {
                robot.deposit.setArmControls(Constants.DEPOSIT_ARM_HIGH);
            }

            if (slidesHome){
                robot.deposit.setSlidesControls(0);
            } else {
                robot.deposit.setSlidesControls(slidesExtendDist);
            }

            robot.deposit.setTurretTheta(turretTheta);

            if (gamepad1.b && depositToggle == false){
                depositToggle = true;
                if (depositGatePos == 0){
                    robot.deposit.close();
                    depositGatePos ++;
                } else if (depositGatePos == 1){
                    robot.deposit.hold();
                    depositGatePos ++;
                } else if (depositGatePos == 2){
                    robot.deposit.open();
                    depositGatePos = 0;
                }
            }
            if (!gamepad1.b){
                depositToggle = false;
            }
        }
    }
}
