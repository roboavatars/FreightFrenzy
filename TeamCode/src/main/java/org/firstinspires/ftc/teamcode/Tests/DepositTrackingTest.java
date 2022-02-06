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

@TeleOp(name = "0 0 0 0 deposit tracking test")
@Config
public class DepositTrackingTest extends LinearOpMode {
    private boolean armHome = true;
    private boolean slidesHome = true;
    private boolean depositToggle = false;
    private int depositGatePos = 0;

    private Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this, 102, 9, 0, false, true);
        robot.intake.home();

        waitForStart();

        while (opModeIsActive()) {
            robot.depositAllianceHub(Deposit.DepositHeight.HIGH);

            // Deposit Gate
            if (gamepad1.b && depositToggle == false) {
                depositToggle = true;
                if (depositGatePos == 0) {
                    robot.deposit.close();
                    depositGatePos ++;
                } else if (depositGatePos == 1) {
                    robot.deposit.hold();
                    depositGatePos ++;
                } else if (depositGatePos == 2) {
                    robot.deposit.open();
                    depositGatePos = 0;
                }
            }
            if (!gamepad1.b) {
                depositToggle = false;
            }

            robot.drivetrain.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            robot.update();

            addPacket("1 arm ticks", robot.deposit.getArmPosition());
            addPacket("2 slide inches", robot.deposit.getSlidesDistInches());
            addPacket("3 slide ticks", robot.deposit.getSlidesPosition());
            addPacket("4 slides target ticks", robot.deposit.targetSlidesTicks);
            addPacket("5 arm home", armHome);
            addPacket("6 slide home", slidesHome);
            addPacket("7 current theta", robot.deposit.getTurretTheta());
        }
    }
}
