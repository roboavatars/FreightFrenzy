package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@Disabled
@TeleOp(name = "deposit tracking test")
@Config
public class DepositTrackingTest extends LinearOpMode {
    private boolean armHome = true;
    private boolean slidesHome = true;
    private boolean depositToggle = false;
    private boolean depositGateHold = true;

    private Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this, 102, 9, 0, false, true);
        robot.intake.home();

        waitForStart();

        while (opModeIsActive()) {
            robot.cycleHub = Robot.DepositTarget.allianceHigh;

            // Deposit Gate
            if (gamepad1.b && depositToggle == false) {
                depositToggle = true;
                if (depositGateHold) {
                    robot.deposit.hold();
                } else {
                    robot.deposit.open();
                }
                depositGateHold = !depositGateHold;
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
            addPacket("7 current theta", robot.turret.getTurretTheta());
        }
    }
}
