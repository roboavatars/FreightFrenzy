package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Localization.AntiTip;
import org.firstinspires.ftc.teamcode.RobotClasses.Drivetrain;

//Config in Constants.java
@TeleOp(name = "AntiTip Test")
@Disabled
public class AntiTipTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AntiTip antiTip = new AntiTip(this);
        Drivetrain dt = new Drivetrain(this, 0, 0, 0);

        waitForStart();
        while (opModeIsActive()){
            double [] adjustedControls = antiTip.update();

            dt.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            dt.updatePose();

            addPacket("axis 1", antiTip.currentOrientation[0]);
            addPacket("axis 2", antiTip.currentOrientation[1]);
            addPacket("tipping", antiTip.tipping);
            addPacket("x controls", adjustedControls[0]);
            addPacket("y controls", adjustedControls[1]);
            sendPacket();
        }
    }
}
